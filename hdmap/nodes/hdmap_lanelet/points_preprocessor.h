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

#ifndef POINTS_PREPROCESSOR_H
#define POINTS_PREPROCESSOR_H

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


class PointsProcessor
{
private:
    /* data */
    const int OK = 0;
    const int FAILED = -1;

public:

    int AggreateMap(std::string pcd_file_path, std::string pose6d_file_path);

    PointsProcessor(/* args */);
    ~PointsProcessor();
};



#endif
