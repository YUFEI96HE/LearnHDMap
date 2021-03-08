/*
 * @file: 
 * 
 * @brief: 
 * 
 * @author: heyufei
 * 
 * @data: 2021-03-03 17:02
 * 
 */

//ros
#include <ros/ros.h>

//custom
#include "hdmap/MapBin.h"
#include "../lanelet2_extension/extension.h"

//lanelet2
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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

#include <iostream>
#include <fstream>

double distance_from_curb; //m
double min_distance;       //m
std::vector<std::vector<geometry_msgs::Point>> trajectory;
std::string LaneletMapTopic;
std::string traj_json_save_dir;
ros::Publisher traj_pub;
int save_count = 0;

void HDMapCB(hdmap::MapBin msg);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Generate Trajectory");
    ros::NodeHandle nh;

    nh.getParam("/gen_trajectory/min_distance", min_distance);
    nh.getParam("/gen_trajectory/distance_from_curb", distance_from_curb);
    nh.getParam("/gen_trajectory/traj_json_save_dir", traj_json_save_dir);

    std::cout << min_distance << " " << distance_from_curb << " " << traj_json_save_dir << std::endl;
    ros::Subscriber lanelet_map_sub = nh.subscribe("/hdmap_lanelet/hdmap_lanelet_bin", 1, HDMapCB);
    traj_pub = nh.advertise<geometry_msgs::PoseArray>("/sweep_curb_traj", 1);

    ros::spin();

    return 0;
}

void HDMapCB(hdmap::MapBin msg)
{
    lanelet::LaneletMapPtr this_lanelet_map(new lanelet::LaneletMap);
    lanelet::extension::fromBinMsg(msg, this_lanelet_map);

    ROS_INFO("LaneletMap Loaded");

    trajectory.resize(2);

    for (lanelet::ConstLanelet this_lanelet : this_lanelet_map->laneletLayer)
    {
        std::cout << this_lanelet.id() << std::endl;
        ROS_ERROR("000");
        if (!this_lanelet.hasAttribute("local_name"))
        {
            continue;
        }
        if (this_lanelet.attribute("local_name").value() == "road-1")
        {
            ROS_ERROR("111");
            // lanelet::ConstLineString3d this_left_line = this_lanelet.leftBound3d();
            lanelet::ConstLineString3d this_right_line_1 = this_lanelet.rightBound3d();
            // lanelet::ConstLineString3d this_center_line = this_lanelet.centerline3d();

            lanelet::ConstPoint3d sp_1 = this_right_line_1.front();
            lanelet::ConstPoint3d ep_1 = this_right_line_1.back();

            std::cout << sp_1.x() << " " << sp_1.y() << " " << ep_1.x() << " " << ep_1.y() << std::endl;

            geometry_msgs::Point gmp;
            gmp.x = sp_1.x();
            gmp.y = sp_1.y() + distance_from_curb;
            // gmp.x = 0;
            // gmp.y = 0;

            // std::cout << gmp.x << " " << gmp.y << std::endl;

            // trajectory.at(0).push_back(gmp);
            std::cout << std::abs(ep_1.x() - sp_1.x()) << std::endl;
            std::cout << min_distance << std::endl;

            double rate = std::abs(ep_1.x() - sp_1.x()) / min_distance;
            double miny = std::abs(ep_1.y() - sp_1.y()) / rate;

            // while (gmp.x < 0)
            // {
            //     gmp.x += min_distance;

            // }

            std::cout << "Rate: " << rate << std::endl;

            geometry_msgs::Point fisrt_point;

            for (int i = 0; i < rate; ++i)
            {
                gmp.x = gmp.x + min_distance;
                gmp.y = gmp.y + miny;

                if (gmp.x < 0)
                {
                    // gmp.x = gmp.x + min_distance;
                    // gmp.y = gmp.y + miny;
                    continue;
                }
                // else if (gmp.x >= 0)
                // {
                //     trajectory.at(0).push_back(gmp);
                //     gmp.x = trajectory.at(0).at(i).x + min_distance;
                //     gmp.y = trajectory.at(0).at(i).y + miny;
                // }
                trajectory.at(0).push_back(gmp);

                // gmp.x += min_distance;
                // gmp.y += miny;
                // trajectory.at(0).push_back(gmp);
            }
        }
        else if (this_lanelet.attribute("local_name").value() == "road-2")
        {
            ROS_ERROR("222");
            // lanelet::ConstLineString3d this_left_line = this_lanelet.leftBound3d();
            lanelet::ConstLineString3d this_right_line_2 = this_lanelet.rightBound3d();
            // lanelet::ConstLineString3d this_center_line = this_lanelet.centerline3d();

            lanelet::ConstPoint3d sp_2 = this_right_line_2.front();
            lanelet::ConstPoint3d ep_2 = this_right_line_2.back();

            ROS_INFO("ROAD-2");
            std::cout << sp_2.x() << " " << sp_2.y() << std::endl;
            std::cout << ep_2.x() << " " << ep_2.y() << std::endl;

            geometry_msgs::Point gmp;
            gmp.x = sp_2.x();
            gmp.y = sp_2.y() - distance_from_curb;

            std::cout << trajectory.at(1).size() << std::endl;

            trajectory.at(1).push_back(gmp);

            double rate = std::abs(ep_2.x() - sp_2.x()) / min_distance;
            double miny = std::abs(ep_2.y() - sp_2.y()) / rate;

            std::cout << "rate: " << rate << std::endl;
            std::cout << "miny: " << miny << std::endl;

            for (int i = 0; i < rate; ++i)
            {
                gmp.x = trajectory.at(1).back().x - min_distance;
                gmp.y = trajectory.at(1).back().y - miny;
                trajectory.at(1).push_back(gmp);
            }
        }
        else
        {
            break;
        }
    }

    ROS_INFO("Traj size: %i", trajectory.size());

    geometry_msgs::PoseArray traj_msg;

    Json::Value root;
    root["name"] = Json::Value("sweep-curb-traj");
    Json::Value pose;
    for (auto p = trajectory.begin(); p != trajectory.end(); p++)
    {
        std::vector<geometry_msgs::Point> road_vec = *p;

        for (auto g = road_vec.begin(); g != road_vec.end(); g++)
        {
            geometry_msgs::Point temp_gp = *g;
            geometry_msgs::Pose temp_pose;
            temp_pose.position.x = temp_gp.x;
            temp_pose.position.y = temp_gp.y;
            temp_pose.position.z = 0;
            traj_msg.poses.push_back(temp_pose);

            Json::Value position;
            position["x"] = temp_pose.position.x;
            position["y"] = temp_pose.position.y;
            pose.append(position);
            // root["traj_seq"].append(temp_pose.position.y);
        }
        // std::cout << this_point << std::endl;
    }
    root["traj"] = pose;
    std::cout << "StyledWriter:" << std::endl;
    Json::StyledWriter sw;
    // std::cout << sw.write(root) << std::endl;
    if (save_count == 0)
    {
        std::ofstream os(traj_json_save_dir + "traj.json", std::ios::out);
        os << sw.write(root);
        os.close();
        save_count++;
    }

    traj_msg.header.frame_id = "map";
    traj_msg.header.stamp = ros::Time().now();
    traj_pub.publish(traj_msg);
    // ros::shutdown();
}
