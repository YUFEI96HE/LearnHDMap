#include "points_preprocessor.h"

PointsProcessor::PointsProcessor(/* args */){}

PointsProcessor::~PointsProcessor(){}



int PointsProcessor::AggreateMap(std::string rgb_pcd_file_path, std::string pose6d_file_path)
{
    pcl::PointCloud<PointTPose>::Ptr pose6d;
    pose6d.reset(new pcl::PointCloud<PointTPose>());
    pcl::io::loadPCDFile(pose6d_file_path, *pose6d);
    ROS_WARN("aggreate map find :%d 6d_key_pose", pose6d->points.size());
    int intervel = 5;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr save_map(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::string tmp_pcd_path;
    for (int i = 0; i < pose6d->points.size(); i++)
    {
        if (!((i + 1) % intervel == 0))
        {
            continue;
        }

        tmp_pcd_path = rgb_pcd_file_path + "/" + std::to_string(i + 1) + ".pcd";

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp3(new pcl::PointCloud<pcl::PointXYZRGB>());

        pcl::io::loadPCDFile(tmp_pcd_path, *tmp);

        for (const auto &p : tmp->points)
        {
            float r = p.x * p.x + p.y * p.y;
            if (r > 1 && r < 30)
            {
                tmp3->points.push_back(p);
            }
        }
        pcl::transformPointCloud(*tmp3, *tmp2, tf_b2l_);
        tmp2 = transformPointCloud(tmp2, pose6d->points[i]);
        *save_map += *tmp2;
    }
    // save_map->width = save_map->points.size();
    // save_map->height = 1;
    // save_map->is_dense = false;
    pcl::io::savePCDFile(rgb_pcd_file_path + "/aggreate_map.pcd", *save_map);
    ROS_WARN("Save map. points size: %d", save_map->points.size());
}