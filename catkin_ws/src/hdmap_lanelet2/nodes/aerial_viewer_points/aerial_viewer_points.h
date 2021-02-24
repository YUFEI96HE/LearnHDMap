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

#ifndef AERIAL_VIEWER_POINTS_H
#define AERIAL_VIEWER_POINTS_H

#include <stdio.h>
#include <iostream>
#include "fstream"
#include "string"
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "image_preprocessor.h"

#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件



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


    std::string param_main_path;
    std::string current_image_path;
    std::string current_pcd_path;

    std::string param_image_id;
    std::string param_lidar_id;

    cv::Mat current_image;
    cv::Mat current_undistorted_img;
    pcl::PointCloud<pcl::PointXYZ> current_pcd;

    float tf_b2l_x;
    float tf_b2l_y;
    float tf_b2l_z;
    float tf_b2l_roll;
    float tf_b2l_pitch;
    float tf_b2l_yaw;

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

    // void PerspectiveTransform();
    // void GenerateAerialViewer();
    // void GenerateAerialMap();

public:
    AerialViewerPoints(/* args */);
    ~AerialViewerPoints();
};

#endif