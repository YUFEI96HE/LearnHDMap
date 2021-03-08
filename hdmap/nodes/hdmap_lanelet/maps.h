/*
 * @file: 
 * 
 * @brief: 
 * 
 * @author: heyufei
 * 
 * @data: 2021-03-03 11:40
 * 
 */

#ifndef MAPS_HEADER
#define MAPS_HEADER

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

#include "yaml-cpp/yaml.h"

#include <lanelet2_io/Io.h>
#include <boost/filesystem.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Units.h>

#include <lanelet2_examples/internal/ExampleHelpers.h> //example工具类

const int OK = 0;
const int FAILED = -1;

class MapsCostmap
{
private:
    /* data */

public:
    int AggreateMap(std::string pcd_file_path, std::string pose6d_file_path);

    PointsProcessor(/* args */);
    ~PointsProcessor();
};

class MapsLaneletMap
{
private:
public:
    lanelet::LaneletMapConstPtr this_lanelet_map;
    
};

class MapsPCD
{
private:
public:
};

#endif