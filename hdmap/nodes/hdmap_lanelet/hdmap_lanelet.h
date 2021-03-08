/*
 * @file: 
 * 
 * @brief: 
 * 
 * @author: heyufei
 * 
 * @data: 2021-02-02 14:56
 * 
 */

#ifndef HDMAP_LANELET_H
#define HDMAP_LANELET_H

#include <stdio.h>
#include <iostream>
#include "fstream"
#include "string"
#include <vector>
#include <assert.h>
#include <climits>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>



#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "image_preprocessor.h"

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

#include <iostream>                         //标准输入输出流
#include <pcl/io/pcd_io.h>                  //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h>                //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h> //点云查看窗口头文件

#include "hdmap/MapBin.h"
#include "yaml-cpp/yaml.h"
#include "../lanelet2_extension/extension.h"


#include <boost/filesystem.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Units.h>
//#include <lanelet2_examples/internal/ExampleHelpers.h> //example工具类

using namespace lanelet;
using namespace lanelet::units::literals;

struct BasePoint
{
    cv::Point2d position2d;
    cv::Point3d position3d;
};

struct MapElements
{
    std::vector<std::pair<int, lanelet::Lanelet>> roads;
    std::vector<std::pair<int, lanelet::Area>> dzones;
};

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

using PointTPose = PointXYZIRPYT;

class AerialViewerPoints
{
private:
    /* data */
    const int OK = 0;
    const int FAILED = -1;

    class ImagePreprocessor ImgPro;

    ros::NodeHandle nh_private;

    ros::Publisher pub_fusion_cloud;
    ros::Publisher map_bin_pub;

    MapElements MyLaneletMap;

    std::string this_node_path;

    std::string param_main_path;
    std::string current_image_path;
    std::string current_pcd_path;
    std::string current_json_path;

    std::vector<cv::Point> right_side_polygon;
    std::vector<cv::Point> decel_polygon;

    std::string param_image_id;
    std::string param_lidar_id;

    std::string param_lanelet_path_in;
    std::string param_lanelet_path_out;

    float param_origin_point_x;
    float param_origin_point_y;

    std::vector<std::pair<cv::Point, cv::Point>> left_side;
    std::vector<std::pair<cv::Point, cv::Point>> right_side;

    cv::Mat current_image;
    cv::Mat current_undistorted_img;
    pcl::PointCloud<pcl::PointXYZ> current_pcd;
    cv::Point2i start_point;

    float tf_b2l_x;
    float tf_b2l_y;
    float tf_b2l_z;
    float tf_b2l_roll;
    float tf_b2l_pitch;
    float tf_b2l_yaw;

    int param_costmap_height;
    int param_costmap_width;

    double param_height_limit;
    double param_resolution;

    std::string param_6d_path;

    tf::TransformListener transform_listener;
    std::string calibration_file;
    int param_data_count;

    void InitROS();

    void MainLoop();

    int LoadDividedData(int count);

    void ColoringPoints();

    tf::StampedTransform FindTransform(const std::string &target_frame, const std::string source_frame);

    pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform);

    tf::Transform GetTransformFromYAML();

    int AggreateMap(std::string rgb_pcd_file_path, std::string pose6d_file_path);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr TransformRGBPoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in, const PointTPose &trans);

    int PCDViewer(std::string pcd_path);

    void LoadJsonLabel(std::string json_file);

    void DrawLabel();

    int Generate2DMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud);

    geometry_msgs::Point Point2Dto3D(cv::Point cv_point);

    cv::Point2i Point3Dto2D(pcl::PointXYZRGB pcl_point, cv::Point start_point);

    cv::Vec3b GetRGB(pcl::PointXYZRGB pcl_point);
    // void PerspectiveTransform();
    // void GenerateAerialViewer();
    // void GenerateAerialMap();

    // void BuildLaneletMap(std::vector<cv::Point2i> line_1,
    //                      std::vector<cv::Point2i> line_2,
    //                      std::vector<cv::Point2i> line_3);

    int LoadMapBins(std::string bin_path);

    void BuildLaneletMap(std::vector<std::pair<cv::Point2d, cv::Point2d>> road_point_vec);

    void ReadLanelet(std::string path_in, float origin_x, float origin_y);

    void WriteLanelet(std::string path_out, lanelet::LaneletMapPtr lanelet_map_s);

    float calcuDistance(uchar *ptr, uchar *ptrCen, int cols);

    cv::Mat MaxMinDisFun(cv::Mat data, float Theta, std::vector<int> centerIndex);

    lanelet::Point3d CreateLanletPoint(cv::Point2d this_cv_point, std::string its_name);

public:
    AerialViewerPoints(/* args */);
    ~AerialViewerPoints();
};

#endif
