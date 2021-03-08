/*
 * @file: 
 * 
 * @brief: 
 * 
 * @author: heyufei
 * 
 * @data: 2021-03-04 18:07
 * 
 */

#include <cstdio>
#include <algorithm>
#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

double param_resolution; // 0.05
int param_cell_width;    //  cvwidth / resolution
int param_cell_height;   //  cvheight / resolution
double param_map_width;
double param_map_height;
double param_offset_x;
double param_offset_y;
double param_offset_z;
double param_roll;
double param_pitch;
double param_yaw;

double param_height_limit;
double param_car_width;
double param_car_height;
double param_filter_bool;

bool SAVEDONE = false;

// double param_cost_base;

std::string param_save_dir;

ros::Publisher costmap_oc_pub;

std::vector<int> GenCostmapVector(cv::Mat &costmap_png_i);
void GenCostmapOC();
void SaveCostmapOC();
geometry_msgs::Point Point2Dto3D(cv::Point cv_point);

// 318, 326

cv::Point start_point(318, 326);
geometry_msgs::Point start_point_3d;

geometry_msgs::Point Point2Dto3D(cv::Point cv_point)
{
    geometry_msgs::Point ros_point;
    ros_point.x = (cv_point.x - start_point.x) * param_resolution;
    ros_point.y = (start_point.y - cv_point.y) * param_resolution;
    ros_point.z = 0;

    return ros_point;
}

std::vector<int> GenCostmapVector(cv::Mat &costmap_png_i)
{
    std::vector<int> costmap_vec(param_cell_width * param_cell_height, 0);
    double map_center_x = param_map_width / 2.0 - param_offset_x;
    double map_center_y = param_map_height / 2.0 - param_offset_y;
    for (int r = 0; r < param_map_height; r++)
    {
        for (int c = 0; c < param_map_width; c++)
        {
            if (r < 2 || r > param_map_height - 2)
            {
                costmap_vec[r * param_map_width + c] = -1;
            }
            else if (c < 2 || c > param_map_width - 2)
            {
                costmap_vec[r * param_map_width + c] = -1;
            }
        }
    }

    // uchar *p;
    for (int row = 0; row < costmap_png_i.rows; row++)
    {
        for (int col = 0; col < costmap_png_i.cols; col++)
        {
            geometry_msgs::Point this_point = Point2Dto3D(cv::Point(col, row));
            int grid_x = (this_point.x + map_center_x) / param_resolution;
            int grid_y = (this_point.y + map_center_y) / param_resolution;
            if (grid_x < 0 || grid_x >= param_cell_width || grid_y < 0 || grid_y >= param_cell_height)
            {
                continue;
            }

            int index = param_cell_width * grid_y + grid_x;

            if ((int)costmap_png_i.at<uchar>(row, col) > 100)
            {
                costmap_vec[index] = 0;
            }
            else
            {
                costmap_vec[index] = -1;
            }
        }
    }

    return costmap_vec;
}

void GenCostmapOC(cv::Mat &costmap_png, nav_msgs::OccupancyGrid &costmap_oc)
{
    geometry_msgs::Point max_point;
    geometry_msgs::Point min_point;

    max_point.x = 0;
    max_point.y = 0;
    min_point.x = 0;
    min_point.y = 0;

    for (int r = 0; r < costmap_png.rows; r++)
    {
        for (int c = 0; c < costmap_png.cols; c++)
        {
            geometry_msgs::Point this_point = Point2Dto3D(cv::Point(c, r));
            if (this_point.x > max_point.x)
            {
                max_point.x = this_point.x;
            }
            if (this_point.y > max_point.y)
            {
                max_point.y = this_point.y;
            }
            if (this_point.x < min_point.x)
            {
                min_point.x = this_point.x;
            }
            if (this_point.y < min_point.y)
            {
                min_point.y = this_point.y;
            }
        }
    }

    std::cout << "max_point.x: " << max_point.x << "  "
              << "max_point.y: " << max_point.y << "  "
              << "min_point.x: " << min_point.x << "  "
              << "min_point.y: " << min_point.y << std::endl;

    param_map_width = std::max((float)std::abs(max_point.x - min_point.x), (float)20.0);
    param_map_height = std::max((float)std::abs(max_point.y - min_point.y), (float)20.0);

    std::cout << "map_width: " << param_map_width << "  "
              << "map_height: " << param_map_height << std::endl;

    param_cell_width = static_cast<int>(std::ceil(param_map_width / param_resolution));
    param_cell_height = static_cast<int>(std::ceil(param_map_height / param_resolution));
    param_offset_x = min_point.x + 0.5 * param_map_width;
    param_offset_y = min_point.y + 0.5 * param_map_height;

    tf::Quaternion tmp_q;
    tmp_q.setRPY(param_roll, param_pitch, param_roll);

    costmap_oc.info.resolution = param_resolution;
    costmap_oc.info.width = param_cell_width;
    costmap_oc.info.height = param_cell_height;
    costmap_oc.info.origin.position.x = (-1) * param_map_width / 2.0 + param_offset_x;
    costmap_oc.info.origin.position.y = (-1) * param_map_height / 2.0 + param_offset_y;
    costmap_oc.info.origin.position.z = param_offset_z;
    costmap_oc.info.origin.orientation.x = tmp_q.getX();
    costmap_oc.info.origin.orientation.y = tmp_q.getY();
    costmap_oc.info.origin.orientation.z = tmp_q.getZ();
    costmap_oc.info.origin.orientation.w = tmp_q.getW();

    std::vector<int> costmap_vec = GenCostmapVector(costmap_png);
    costmap_oc.data.insert(costmap_oc.data.end(), costmap_vec.begin(), costmap_vec.end());
}

void SaveCostmapOC()
{
    nav_msgs::OccupancyGrid costmap_oc;
    cv::Mat src_img = cv::imread("/home/heyufei/YUFEI96HE/LearnHDMap/catkin_ws/src/hdmap/maps/costmap.png", CV_8UC1);
    cv::Rect roi = cv::Rect(5, 5, src_img.size().width - 10, src_img.size().height - 10);
    cv::Mat costmap_png = src_img(roi);
    GenCostmapOC(costmap_png, costmap_oc);

    if (!SAVEDONE)
    {

        // save pgm yaml
        ROS_INFO("Received a %d X %d map @ %.3f m/pix",
                 costmap_oc.info.width,
                 costmap_oc.info.height,
                 costmap_oc.info.resolution);

        std::string pgm_save_dir = param_save_dir + "map.pgm";
        std::string yaml_save_dir = param_save_dir + "map.yaml";

        std::cout << "pgm save dir: " << param_save_dir << std::endl;
        std::cout << "yaml save dir: " << yaml_save_dir << std::endl;

        FILE *out = fopen(pgm_save_dir.c_str(), "w");
        if (!out)
        {
            ROS_ERROR("Couldn't save map file to %s", pgm_save_dir.c_str());
        }

        fprintf(out, "P5\n# CREATOR: GenCostmap.cpp %.3f m/pix\n%d %d\n255\n",
                costmap_oc.info.resolution, costmap_oc.info.width, costmap_oc.info.height);

        for (unsigned int y = 0; y < costmap_oc.info.height; y++)
        {
            for (unsigned int x = 0; x < costmap_oc.info.width; x++)
            {
                unsigned int i = x + (costmap_oc.info.height - y - 1) * costmap_oc.info.width;
                if (costmap_oc.data[i] == -1)
                {
                    fputc(000, out);
                }
                else if (costmap_oc.data[i] == 0)
                {
                    fputc(255, out);
                }
                else
                {
                    fputc(105, out);
                }
            }
        }
        fclose(out);

        ROS_INFO("Writing map occupancy data to %s", yaml_save_dir.c_str());
        FILE *yaml = fopen(yaml_save_dir.c_str(), "w");

        geometry_msgs::Quaternion orientation = costmap_oc.info.origin.orientation;

        double yaw, pitch, roll;
        tf::Matrix3x3(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw);
        fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
                yaml_save_dir.c_str(), costmap_oc.info.resolution, costmap_oc.info.origin.position.x, costmap_oc.info.origin.position.y, yaw);
        fclose(yaml);
        ROS_INFO("Done\n");
        SAVEDONE = true;
    }

    costmap_oc.header.frame_id = "map";
    costmap_oc.header.stamp = ros::Time().now();
    costmap_oc_pub.publish(costmap_oc);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Generate OccupancyGrid Costmap");

    // ros::NodeHandle nh;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param<double>("resolution", param_resolution, 0.05);
    nh_private.param<double>("map_width", param_map_width, 50.);
    nh_private.param<double>("map_height", param_map_height, 50.);
    nh_private.param<double>("offset_x", param_offset_x, 0.0);
    nh_private.param<double>("offset_y", param_offset_y, 0.0);
    nh_private.param<double>("offset_z", param_offset_z, 0.0);
    nh_private.param<double>("height_limit", param_height_limit, 0.1);
    nh_private.param<double>("car_width", param_car_width, 0.6);
    nh_private.param<double>("car_length", param_car_height, 1.2);
    // nh.param<double>("cost_base", param_cost_base, 50);

    nh_private.param<std::string>("save_dir", param_save_dir, "");
    // nh_private.param<double>("roll", param_roll, 0.0);
    // nh_private.param<double>("pitch", param_pitch, 0.0);
    // nh_private.param<double>("yaw", param_yaw, 0.0);

    // nh_private.param<double>("height_limit", param_height_limit, 0.1);
    // nh_private.param<double>("car_width", param_car_width, 0.6);
    // nh_private.param<double>("car_length", param_car_height, 1.2);

    costmap_oc_pub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 10);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        SaveCostmapOC();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
