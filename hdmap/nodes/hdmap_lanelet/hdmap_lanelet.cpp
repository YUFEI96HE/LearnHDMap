#include "hdmap_lanelet.h"

using namespace lanelet;
using namespace lanelet::units::literals;

AerialViewerPoints::AerialViewerPoints(/* args */) : nh_private("~")
{
    InitROS();
}

AerialViewerPoints::~AerialViewerPoints()
{
}

void AerialViewerPoints::InitROS()
{
    ROS_INFO("Aerial_Viewer_Points");
    // ng_provate.param<std::string>("")
    nh_private.param<std::string>("calibration_file", calibration_file, "");
    nh_private.param<std::string>("divided_data_path", param_main_path, "");
    nh_private.param<std::string>("pose_6d_path", param_6d_path, "");
    nh_private.param<std::string>("image_id", param_image_id, "");
    nh_private.param<std::string>("lidar_id", param_lidar_id, "");
    nh_private.param<std::string>("laneletMap_path_in", param_lanelet_path_in, "");
    nh_private.param<std::string>("laneletMap_path_out", param_lanelet_path_out, "");
    nh_private.param<int>("data_num", param_data_count, 0);
    nh_private.param<int>("height", ImgPro.height, 0);
    nh_private.param<int>("width", ImgPro.width, 0);
    nh_private.param<int>("costmap_height", param_costmap_height, 0);
    nh_private.param<int>("costmap_width", param_costmap_width, 0);
    nh_private.param<float>("fx", ImgPro.fx, 0.0);
    nh_private.param<float>("fy", ImgPro.fy, 0.0);
    nh_private.param<float>("cx", ImgPro.cx, 0.0);
    nh_private.param<float>("cy", ImgPro.cy, 0.0);
    nh_private.param<float>("k1", ImgPro.k1, 0.0);
    nh_private.param<float>("k2", ImgPro.k2, 0.0);
    nh_private.param<float>("tf_b2l_x", tf_b2l_x, 0.0);
    nh_private.param<float>("tf_b2l_y", tf_b2l_y, 0.0);
    nh_private.param<float>("tf_b2l_z", tf_b2l_z, 0.0);
    nh_private.param<float>("tf_b2l_roll", tf_b2l_roll, 0.0);
    nh_private.param<float>("tf_b2l_pitch", tf_b2l_pitch, 0.0);
    nh_private.param<float>("tf_b2l_yaw", tf_b2l_yaw, 0.0);
    nh_private.param<float>("tf_b2l_yaw", tf_b2l_yaw, 0.0);
    nh_private.param<float>("origin_point_x", param_origin_point_x, 0.0);
    nh_private.param<float>("origin_point_y", param_origin_point_y, 0.0);
    nh_private.param<double>("resolution", param_resolution, 0.0);

    pub_fusion_cloud = nh_private.advertise<sensor_msgs::PointCloud2>("fusison_output_topic", 1);

    MainLoop();
}

void AerialViewerPoints::ReadLanelet(std::string path_in, float origin_x, float origin_y)
{
    lanelet::Origin origin{{origin_x, origin_y}}; //设置地图的初始点

    lanelet::LaneletMapPtr lanelet_map = lanelet::load(path_in, origin);

    // lanelet_map.get()
}

int AerialViewerPoints::LoadMapBins(std::string bin_path)
{
    std::string lanelet_map_path;

    boost::filesystem::path map_path(bin_path);

    if (boost::filesystem::is_regular_file(map_path))
    {
        lanelet_map_path = map_path.generic_string();
        std::cout << "lanelet_map_path-generic_string: " << lanelet_map_path << std::endl;
    }
    else if (boost::filesystem::is_directory(map_path))
    {
        std::vector<boost::filesystem::path> map_path_list;
        for (const boost::filesystem::path &entry : boost::make_iterator_range(boost::filesystem::directory_iterator(map_path), {}))
        {
            if (boost::filesystem::is_regular_file(entry))
            {
                map_path_list.push_back(entry);
            }
        }
        if (map_path_list.size() > 0)
        {
            auto min_it = std::min_element(map_path_list.begin(), map_path_list.end(),
                                           [](const boost::filesystem::path &a, const boost::filesystem::path &b) {
                                               return a.filename().generic_string() < b.filename().generic_string();
                                           });
            lanelet_map_path = (*min_it).generic_string();
        }
        else
        {
            lanelet_map_path = "";
        }
    }
    if (lanelet_map_path == "")
    {
        ROS_ERROR("[lanelet2_map_loader] File name is not specified or wrong. [%s]", lanelet_map_path.c_str());
        return FAILED;
    }

    ROS_INFO("[lanelet2_map_loader] Will load %s", lanelet_map_path.c_str());

    lanelet::LaneletMapPtr this_lanelet_map = lanelet::load(lanelet_map_path);

    // lanelet::utils::overwriteLaneletsCenterline(this_lanelet_map, false);

    // std::string format_version, map_version;
    // lanelet::io_handlers::OsmParser::parseVersions(lanelet_map_path, &format_version, &map_version);

    map_bin_pub = nh_private.advertise<hdmap::MapBin>("hdmap_lanelet_bin", 1, true);

    hdmap::MapBin map_bin_msg;
    map_bin_msg.header.stamp = ros::Time::now();
    map_bin_msg.header.frame_id = "map";
    // map_bin_msg.format_version = format_version;
    // map_bin_msg.map_version = map_version;
    lanelet::extension::toBinMsg(this_lanelet_map, &map_bin_msg);

    // !!!EDITION
    // while (true)
    // {
    //     map_bin_pub.publish(map_bin_msg);
    //     sleep(1);
    // }
}

void AerialViewerPoints::WriteLanelet(std::string path_out, lanelet::LaneletMapPtr lanelet_map_s)
{
    lanelet::write(path_out, *lanelet_map_s);
}

lanelet::Point3d AerialViewerPoints::CreateLanletPoint(cv::Point2d cv_point, std::string its_name)
{
    geometry_msgs::Point point_3d = Point2Dto3D(cv_point);
    lanelet::Point3d this_point(lanelet::utils::getId(), point_3d.x, point_3d.y, point_3d.z);
    this_point.attributes()["local_2d_x"] = cv_point.x;
    this_point.attributes()["local_2d_y"] = cv_point.y;
    this_point.attributes()["local_3d_x"] = point_3d.x;
    this_point.attributes()["local_3d_y"] = point_3d.y;
    this_point.attributes()["local_3d_z"] = point_3d.z;
    this_point.attributes()["local_name"] = its_name;
    return this_point;
}

// input: blw mid top dzone
void AerialViewerPoints::BuildLaneletMap(std::vector<std::pair<cv::Point2d, cv::Point2d>> point_vec)
{
    // dzones jiansudai
    lanelet::Point3d dzone_1_point_1 = CreateLanletPoint(point_vec.at(3).second, "dzone-1-point-1"); //max_y
    lanelet::Point3d dzone_1_point_2 = CreateLanletPoint(point_vec.at(3).first, "dzone-1-point-2");  //min_y_cv
    lanelet::LineString3d dzone_1(lanelet::utils::getId(), {dzone_1_point_1, dzone_1_point_2});
    dzone_1.attributes()["local_name"] = "dzone-1";

    // roads

    // -- road-1  (forward)  right-first in lanelet road
    lanelet::Point3d road_1_crub_1_point_1 = CreateLanletPoint(point_vec.at(0).first, "road-1-crub-1-point-1");  //min_x
    lanelet::Point3d road_1_crub_1_point_2 = CreateLanletPoint(point_vec.at(0).second, "road-1-crub-1-point-2"); //max_x
    lanelet::LineString3d road_1_crub_1(lanelet::utils::getId(), {road_1_crub_1_point_1, road_1_crub_1_point_2});
    road_1_crub_1.attributes()["local_name"] = "road-1-crub-1";

    lanelet::Point3d road_1_crub_2_point_1 = CreateLanletPoint(point_vec.at(2).first, "road-1-crub-2-point-1");  //min_x
    lanelet::Point3d road_1_crub_2_point_2 = CreateLanletPoint(point_vec.at(2).second, "road-1-crub-2-point-2"); //max_x
    lanelet::LineString3d road_1_crub_2(lanelet::utils::getId(), {road_1_crub_2_point_1, road_1_crub_2_point_2});
    road_1_crub_2.attributes()["local_name"] = "road-1-crub-2";
    road_1_crub_2.attributes()[AttributeName::Subtype] = AttributeValueString::Dashed;

    lanelet::Lanelet road_1(lanelet::utils::getId(), road_1_crub_1, road_1_crub_2);
    road_1.attributes()[AttributeName::Type] = AttributeValueString::LineThin;
    road_1.attributes()["local_name"] = "road-1";
    road_1.attributes()["local_direction"] = "work-step-1";
    road_1.setRightBound(road_1_crub_1);
    road_1.setLeftBound(road_1_crub_2);

    // -- road-2  (return)
    lanelet::Point3d road_2_crub_1_point_1 = CreateLanletPoint(point_vec.at(1).first, "road-2-crub-1-point-1");  //max_x
    lanelet::Point3d road_2_crub_1_point_2 = CreateLanletPoint(point_vec.at(1).second, "road-2-crub-1-point-2"); //min_x
    lanelet::LineString3d road_2_crub_1(lanelet::utils::getId(), {road_2_crub_1_point_1, road_2_crub_1_point_2});
    road_2_crub_1.attributes()["local_name"] = "road-2-crub-1";

    lanelet::Point3d road_2_crub_2_point_1 = CreateLanletPoint(point_vec.at(2).second, "road-2-crub-2-point-1");
    lanelet::Point3d road_2_crub_2_point_2 = CreateLanletPoint(point_vec.at(2).first, "road-2-crub-2-point-2");
    lanelet::LineString3d road_2_crub_2(lanelet::utils::getId(), {road_1_crub_2_point_1, road_1_crub_2_point_2});
    road_2_crub_2.attributes()["local_name"] = "road-2-crub-2";
    road_2_crub_2.attributes()[AttributeName::Subtype] = AttributeValueString::Dashed;

    lanelet::Lanelet road_2(lanelet::utils::getId(), road_2_crub_1, road_2_crub_2);
    road_2.attributes()[AttributeName::Type] = AttributeValueString::LineThin;
    road_2.attributes()["local_name"] = "road-2";
    road_2.attributes()["local_direction"] = "work-step-2";
    road_2.setRightBound(road_2_crub_1);
    road_2.setLeftBound(road_2_crub_2);

    lanelet::LaneletMapPtr my_lanelet_map;
    my_lanelet_map->add(dzone_1);
    my_lanelet_map->add(road_1);
    my_lanelet_map->add(road_2);
    ROS_ERROR("BEGIN TO SAVE LANELETMAP");
    lanelet::write(this_node_path + "/maps/my_lanelet_map.osm", *my_lanelet_map);
    lanelet::write(this_node_path + "/maps/my_lanelet_map.bin", *my_lanelet_map);
    // write(tempfile("map.bin"), *map);
    ROS_ERROR("LANELETMAP_SAVING_DONE");
}

void AerialViewerPoints::MainLoop()
{
    //
    std::string nana = "/home/heyufei/YUFEI96HE/LearnHDMap/catkin_ws/src/hdmap/maps/my_lanelet_map.bin";
    LoadMapBins(nana);
    this_node_path = ros::package::getPath("hdmap");
    for (int i = 5; i < param_data_count; i += 5)
    {
        int res = LoadDividedData(i);
        if (res == FAILED)
        {
            continue;
        }
        else if (res == OK)
        {
            std::cout << "get: " << i << std::endl;
            // ImgPro.UndistortImage(current_image, current_undistorted_img);
            // cv::imshow("current_undistorted_img", current_undistorted_img);
            // cv::waitKey(300);
        }
        else
        {
            std::cout << "ERROR IN MAINLOOP" << std::endl;
        }
        DrawLabel();
        ColoringPoints();
    }
    AggreateMap(param_main_path, param_6d_path);

    // ReadLanelet(param_lanelet_path_in, param_origin_point_x, param_origin_point_y);
}

// int AerialViewerPoints::PCDViewer(std::string pcd_path)
// {
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）

// 	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
// 	{
// 		PCL_ERROR("Couldn't read combined pcd file \n"); //文件不存在时，返回错误，终止程序。
// 		return (-1);
// 	}
// 	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//直接创造一个显示窗口
// 	viewer.showCloud(cloud);//在这个窗口显示点云
// 	while (!viewer.wasStopped())
// 	{
// 	}
// 	return (0);
// }

int AerialViewerPoints::LoadDividedData(int count)
{
    right_side_polygon.clear();
    decel_polygon.clear();

    current_image_path = param_main_path + "jpg/" + std::to_string(count) + ".jpg";
    current_pcd_path = param_main_path + "pcd/" + std::to_string(count) + ".pcd";
    current_json_path = param_main_path + "json/" + std::to_string(count) + ".json";

    current_image = cv::imread(current_image_path);
    if (!current_image.data)
    {
        ROS_ERROR("cannot load image file: %s\n", current_image_path);
        return FAILED;
    }
    else if (pcl::io::loadPCDFile<pcl::PointXYZ>(current_pcd_path, current_pcd) == -1) //* load the file
    {
        PCL_ERROR("cannot load pcd file %s\n", current_pcd_path);
        return FAILED;
    }
    Json::Reader reader;
    Json::Value root;

    std::ifstream in(current_json_path, std::ios::binary);

    if (!in.is_open())
    {
        return FAILED;
    }

    if (reader.parse(in, root))
    {
        for (unsigned int i = 0; i < root["shapes"].size(); ++i)
        {
            std::string label = root["shapes"][i]["label"].asString();
            if (label == "left")
            {
                // continue;
            }
            else if (label == "right")
            {
                for (unsigned int x = 0; x < root["shapes"][i]["points"].size(); x++)
                {
                    double this_point_x = root["shapes"][i]["points"][x][0].asDouble();
                    double this_point_y = root["shapes"][i]["points"][x][1].asDouble();

                    cv::Point this_point(round(this_point_x), round(this_point_y));
                    right_side_polygon.push_back(this_point);
                }
            }
            else if (label == "decel")
            {
                for (unsigned int x = 0; x < root["shapes"][i]["points"].size(); x++)
                {
                    double this_point_x = root["shapes"][i]["points"][x][0].asDouble();
                    double this_point_y = root["shapes"][i]["points"][x][1].asDouble();

                    cv::Point this_point(round(this_point_x), round(this_point_y));
                    decel_polygon.push_back(this_point);
                }
            }
            else
            {
                std::cout << "Error json file" << std::endl;
                return FAILED;
            }
        }
    }

    return OK;
}

void AerialViewerPoints::DrawLabel()
{
    // src_img = cv::imread(image_path, cv::IMREAD_COLOR);

    if (current_image.empty())
    {
        std::cout << "Could not read the image: " << current_image_path << std::endl;
    }

    if (right_side_polygon.size() > 0)
    {
        std::cout << "right_size: " << right_side_polygon.size() << std::endl;
        for (unsigned int c = 0; c < right_side_polygon.size(); ++c)
        {
            std::cout << right_side_polygon[c] << std::endl;
        }
        cv::polylines(current_image, right_side_polygon, true, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        std::vector<std::vector<cv::Point>> this_polygon_vector_right;
        this_polygon_vector_right.push_back(right_side_polygon);
        cv::fillPoly(current_image, this_polygon_vector_right, cv::Scalar(0, 0, 255));
    }

    if (decel_polygon.size() > 0)
    {
        std::cout << "decel_size: " << decel_polygon.size() << std::endl;
        for (unsigned int c = 0; c < decel_polygon.size(); ++c)
        {
            std::cout << decel_polygon[c] << std::endl;
        }

        cv::polylines(current_image, decel_polygon, true, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        std::vector<std::vector<cv::Point>> this_polygon_vector_decel;
        this_polygon_vector_decel.push_back(decel_polygon);
        cv::fillPoly(current_image, this_polygon_vector_decel, cv::Scalar(255, 0, 0));
    }

    // cv::imshow("labeled_img", current_image);
    // cv::waitKey(100);
}

void AerialViewerPoints::ColoringPoints()
{
    std::vector<pcl::PointXYZ> cam_cloud(current_pcd.points.size());
    cam_cloud.clear();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    rgb_cloud->points.clear();

    pcl::PointXYZRGB colored_3d_point;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    out_cloud->points.clear();

    // std::cout << "current_point_cloud_size: " << current_pcd.points.size() << std::endl;

    int decel_points = 0;
    int right_points = 0;

    for (size_t i = 0; i < current_pcd.points.size(); i++)
    {
        // std::cout << "in_cloud_x: " << current_pcd.points[i].x << "   "
        //           << "y: " << current_pcd.points[i].y << "   "
        //           << "z: " << current_pcd.points[i].z << std::endl;

        tf::StampedTransform lidar_to_camera_tf = FindTransform(param_lidar_id, param_image_id);
        // tf::Transform lidar_to_camera_tf = GetTransformFromYAML();
        cam_cloud[i] = TransformPoint(current_pcd.points[i], lidar_to_camera_tf);

        // std::cout << "ca_cloud_x: " << cam_cloud[i].x << "   "
        //           << "y: " << cam_cloud[i].y << "   "
        //           << "z: " << cam_cloud[i].z << std::endl;

        // transformed_cloud->points.push_back(cam_cloud[i]);

        // 使用相机内参将三维空间点投影到像素平面 三维点转二维点

        int col = int(cam_cloud[i].x * ImgPro.fx / cam_cloud[i].z + ImgPro.cx); //U
        int row = int(cam_cloud[i].y * ImgPro.fy / cam_cloud[i].z + ImgPro.cy); //V

        // int col = int(cx - cam_cloud[i].y / cam_cloud[i].x * fx);
        // int row = int(cy - cam_cloud[i].z / cam_cloud[i].x * fy);

        // std::cout << "col: " << col << "   " << "row: " << row << std::endl;

        //     //选取在图像范围内的三维点云
        if ((col >= 0) && (col < ImgPro.width) && (row >= 0) && (row < ImgPro.height) && (current_pcd[i].z < param_height_limit))
        // && (current_pcd[i].z > -10))
        // if ((col >= 0) && (row >= 0) && (ImgPro.width / 2 < col) && (col < ImgPro.width) && (ImgPro.height / 2 < row) && (row < ImgPro.height))
        // && (-10 < current_pcd.points[i].z) && (current_pcd.points[i].z < 10))
        {
            // all
            // colored_3d_point.x = current_pcd.points[i].x;
            // colored_3d_point.y = current_pcd.points[i].y;
            // colored_3d_point.z = current_pcd.points[i].z;

            // cv::Vec3b rgb_pixel = current_undistorted_img.at<cv::Vec3b>(row, col);

            // colored_3d_point.r = rgb_pixel[2];
            // colored_3d_point.g = rgb_pixel[1];
            // colored_3d_point.b = rgb_pixel[0];
            // rgb_cloud->points.push_back(colored_3d_point);

            // with label
            cv::Vec3b rgb_pixel = current_image.at<cv::Vec3b>(row, col);
            // cv::Vec3b blue_pixel = (255, 0, 0);
            // cv::Vec3b red_pixel = (0, 0, 255);

            if (rgb_pixel[2] == 255 && rgb_pixel[1] == 0 && rgb_pixel[0] == 0)
            {
                colored_3d_point.x = current_pcd.points[i].x;
                colored_3d_point.y = current_pcd.points[i].y;
                colored_3d_point.z = current_pcd.points[i].z;
                colored_3d_point.r = rgb_pixel[2];
                colored_3d_point.g = rgb_pixel[1];
                colored_3d_point.b = rgb_pixel[0];
                rgb_cloud->points.push_back(colored_3d_point);
                right_points++;
            }
            else if (rgb_pixel[2] == 0 && rgb_pixel[1] == 0 && rgb_pixel[0] == 255)
            {
                colored_3d_point.x = current_pcd.points[i].x;
                colored_3d_point.y = current_pcd.points[i].y;
                colored_3d_point.z = current_pcd.points[i].z;
                colored_3d_point.r = rgb_pixel[2];
                colored_3d_point.g = rgb_pixel[1];
                colored_3d_point.b = rgb_pixel[0];
                rgb_cloud->points.push_back(colored_3d_point);
                decel_points++;
            }
        }
        else
        {
            continue;
        }
    }

    // std::cout << cam_cloud.size() << "  points in range of camera" << std::endl;
    // std::cout << rgb_cloud->size() << "  rgb points in this pcd" << std::endl;

    rgb_cloud->width = rgb_cloud->points.size();
    rgb_cloud->height = 1;
    rgb_cloud->is_dense = false;
    // int num = i * 5 + 5;
    std::string save_name = current_image_path + ".pcd";
    pcl::io::savePCDFile(save_name, *rgb_cloud);

    // std::cout << "CAMERA_POINT_SIZE: " << out_cloud->points.size() << std::endl;

    // sensor_msgs::PointCloud2 out_cloud_msg;
    // pcl::toROSMsg(*out_cloud, out_cloud_msg);
    // out_cloud_msg.header.frame_id = param_lidar_id;
    // out_cloud_msg.header.stamp = ros::Time();
    // pub_fusion_cloud.publish(out_cloud_msg);

    // sensor_msgs::PointCloud2 test_cloud_1;
    // pcl::toROSMsg(*color_cloud, test_cloud_1);
    // test_cloud_1.header.frame_id = point_id;
    // test_cloud_1.header.stamp = ros::Time();
    // test_pointcloud_1.publish(test_cloud_1);

    // -4- color points which match roadside pixels

    // std::cout << count << " files have been saved" << std::endl;
}

tf::StampedTransform AerialViewerPoints::FindTransform(const std::string &target_frame, const std::string source_frame)
{
    tf::StampedTransform transform;
    try
    {
        // ROS_INFO("FIND TRANSFORM");
        // ros::Time(0)指定了时间为0，即获得最新有效的变换。
        // 改变获取当前时间的变换，即改为ros::Time::now(),不过now的话因为监听器有缓存区的原因。一般会出错
        transform_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.1));
        transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_INFO("pixel_point_fusion : %s", ex.what());
    }

    // std::cout << transform->Transform << std::endl;

    // std::cout << "Tx: " << transform.getOrigin().x() << std::endl;
    // std::cout << "Ty: " << transform.getOrigin().y() << std::endl;
    // std::cout << "Tz: " << transform.getOrigin().z() << std::endl;
    // std::cout << "Rx: " << transform.getRotation().x() << std::endl;
    // std::cout << "Ry: " << transform.getRotation().y() << std::endl;
    // std::cout << "Rz: " << transform.getRotation().z() << std::endl;
    // std::cout << "Rw: " << transform.getRotation().w() << std::endl;
    // transform.getHeader().setFrameId("laser");
    // transform.setChildFrameId("usb_cam");

    return transform;
}

tf::Transform AerialViewerPoints::GetTransformFromYAML()
{
    cv::Mat CameraExtrinsicMat;
    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        ROS_ERROR("Cannot open calibration file");
    }
    fs["CameraExtrinsicMat"] >> CameraExtrinsicMat;

    tf::Matrix3x3 rotation_mat;
    double roll = 0, pitch = 0, yaw = 0;
    tf::Quaternion quaternion;
    tf::Transform transform;

    rotation_mat.setValue(CameraExtrinsicMat.at<double>(0, 0), CameraExtrinsicMat.at<double>(0, 1), CameraExtrinsicMat.at<double>(0, 2),
                          CameraExtrinsicMat.at<double>(1, 0), CameraExtrinsicMat.at<double>(1, 1), CameraExtrinsicMat.at<double>(1, 2),
                          CameraExtrinsicMat.at<double>(2, 0), CameraExtrinsicMat.at<double>(2, 1), CameraExtrinsicMat.at<double>(2, 2));

    // rotation
    rotation_mat = rotation_mat.transpose();
    rotation_mat.getRPY(roll, pitch, yaw, 1);
    quaternion.setRPY(roll, pitch, yaw);
    transform.setRotation(quaternion);

    //translation
    transform.setOrigin(tf::Vector3(CameraExtrinsicMat.at<double>(1, 3),
                                    CameraExtrinsicMat.at<double>(2, 3),
                                    -CameraExtrinsicMat.at<double>(0, 3)));
    // static tf::TransformBroadcaster broadcaster;
    // ros::Time time_stamp_of_image;
    // time_stamp_of_image.sec =
    // broadcaster.sendTransform(tf::StampedTransform(transform, timeStamp, target_frame, camera_frame));

    // std::cout << transform << std::endl;

    // std::cout << "Tx: " << transform.getOrigin().x() << std::endl;
    // std::cout << "Ty: " << transform.getOrigin().y() << std::endl;
    // std::cout << "Tz: " << transform.getOrigin().z() << std::endl;
    // std::cout << "Rx: " << transform.getRotation().x() << std::endl;
    // std::cout << "Ry: " << transform.getRotation().y() << std::endl;
    // std::cout << "Rz: " << transform.getRotation().z() << std::endl;
    // std::cout << "Rw: " << transform.getRotation().w() << std::endl;

    return transform;
}

pcl::PointXYZ AerialViewerPoints::TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
{
    // ROS_INFO("TRANSFORM_POINT");
    tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
    tf::Vector3 tf_point_transformed = in_transform * tf_point;
    return pcl::PointXYZ(tf_point_transformed.x(), tf_point_transformed.y(), tf_point_transformed.z());
}

int AerialViewerPoints::AggreateMap(std::string rgb_pcd_file_path, std::string pose6d_file_path)
{
    pcl::PointCloud<PointTPose>::Ptr pose6d;
    pose6d.reset(new pcl::PointCloud<PointTPose>());
    pcl::io::loadPCDFile(pose6d_file_path, *pose6d);
    ROS_WARN("aggreate map find :%d 6d_key_pose", pose6d->points.size());
    // int intervel = 5;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr save_map(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::string tmp_pcd_path;
    for (int i = 0; i < pose6d->points.size(); i += 5)
    {
        tmp_pcd_path = rgb_pcd_file_path + "jpg/" + std::to_string(i) + ".jpg.pcd";
        // std::cout << "rgb_pcd_path: " << tmp_pcd_path << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>());

        if (pcl::io::loadPCDFile(tmp_pcd_path, *tmp) == FAILED)
        {
            continue;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp3(new pcl::PointCloud<pcl::PointXYZRGB>());

        for (const auto &p : tmp->points)
        {
            float r = p.x * p.x + p.y * p.y;
            if (r > 1 && r < 30)
            {
                tmp3->points.push_back(p);
            }
        }

        Eigen::Matrix4f tf_b2l_ = Eigen::Matrix4f::Identity();
        // float roll, pitch, yaw;
        tf_b2l_(0, 3) = tf_b2l_x;
        tf_b2l_(1, 3) = tf_b2l_y;
        tf_b2l_(2, 3) = tf_b2l_z;

        Eigen::AngleAxisf rx(tf_b2l_roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf ry(tf_b2l_pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rz(tf_b2l_yaw, Eigen::Vector3f::UnitZ());
        tf_b2l_.block(0, 0, 3, 3) = (rz * ry * rx).matrix();

        pcl::transformPointCloud(*tmp3, *tmp2, tf_b2l_);
        tmp2 = TransformRGBPoints(tmp2, pose6d->points[i]);
        *save_map += *tmp2;
    }
    // save_map->width = save_map->points.size();
    // save_map->height = 1;
    // save_map->is_dense = false;
    std::string save_path = param_main_path + "/undistorted_aggreate_map.pcd";
    pcl::io::savePCDFile(save_path, *save_map);
    ROS_WARN("Save map. points size: %d", save_map->points.size());

    // robo::map::PointsToCostmap p2m_(nh_private);
    // p2m_.og_map_saver(save_map, param_main_path);

    Generate2DMap(save_map);

    sensor_msgs::PointCloud2 out_cloud_msg;
    pcl::toROSMsg(*save_map, out_cloud_msg);
    out_cloud_msg.header.frame_id = param_lidar_id;
    out_cloud_msg.header.stamp = ros::Time();
    pub_fusion_cloud.publish(out_cloud_msg);

    // PCDViewer(save_path);
    return OK;
}

float AerialViewerPoints::calcuDistance(uchar *ptr, uchar *ptrCen, int cols)
{
    float d = 0.0;
    for (size_t j = 0; j < cols; j++)
    {
        d += (double)(ptr[j] - ptrCen[j]) * (ptr[j] - ptrCen[j]);
    }
    d = sqrt(d);
    return d;
}

/** @brief   最大最小距离算法
 @param data  输入样本数据，每一行为一个样本，每个样本可以存在多个特征数据
 @param Theta 阈值，一般设置为0.5，阈值越小聚类中心越多
 @param centerIndex 聚类中心的下标
 @return 返回每个样本的类别，类别从1开始，0表示未分类或者分类失败
*/
cv::Mat AerialViewerPoints::MaxMinDisFun(cv::Mat data, float Theta, std::vector<int> centerIndex)
{
    double maxDistance = 0;
    int start = 0;                                                        //初始选一个中心点
    int index = start;                                                    //相当于指针指示新中心点的位置
    int k = 0;                                                            //中心点计数，也即是类别
    int dataNum = data.rows;                                              //输入的样本数
                                                                          //vector<int>	centerIndex;//保存中心点
    cv::Mat distance = cv::Mat::zeros(cv::Size(1, dataNum), CV_32FC1);    //表示所有样本到当前聚类中心的距离
    cv::Mat minDistance = cv::Mat::zeros(cv::Size(1, dataNum), CV_32FC1); //取较小距离

    cv::Mat classes = cv::Mat::zeros(cv::Size(1, dataNum), CV_32SC1); //表示类别
    centerIndex.push_back(index);                                     //保存第一个聚类中心

    for (size_t i = 0; i < dataNum; i++)
    {
        uchar *ptr1 = data.ptr<uchar>(i);
        uchar *ptrCen = data.ptr<uchar>(centerIndex.at(0));
        float d = calcuDistance(ptr1, ptrCen, data.cols);
        distance.at<float>(i, 0) = d;
        classes.at<int>(i, 0) = k + 1;
        if (maxDistance < d)
        {
            maxDistance = d;
            index = i; //与第一个聚类中心距离最大的样本
        }
    }

    minDistance = distance.clone();
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    maxVal = maxDistance;
    while (maxVal > (maxDistance * Theta))
    {
        k = k + 1;
        centerIndex.push_back(index); //新的聚类中心
        for (size_t i = 0; i < dataNum; i++)
        {
            uchar *ptr1 = data.ptr<uchar>(i);
            uchar *ptrCen = data.ptr<uchar>(centerIndex.at(k));
            float d = calcuDistance(ptr1, ptrCen, data.cols);
            distance.at<float>(i, 0) = d;
            //按照当前最近临方式分类，哪个近就分哪个类别
            if (minDistance.at<float>(i, 0) > distance.at<float>(i, 0))
            {
                minDistance.at<float>(i, 0) = distance.at<float>(i, 0);
                classes.at<int>(i, 0) = k + 1;
            }
        }
        //查找minDistance中最大值
        cv::minMaxLoc(minDistance, &minVal, &maxVal, &minLoc, &maxLoc);
        index = maxLoc.y;
    }
    return classes;
}


int AerialViewerPoints::Generate2DMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud)
{
    float top_x = 0, low_x = 0, top_y = 0, low_y = 0;
    std::vector<std::vector<float>> array_3d_points(rgb_cloud->points.size());

    for (unsigned int i = 0; i < rgb_cloud->points.size(); ++i)
    {
        if (rgb_cloud->points.at(i).x > top_x)
        {
            top_x = rgb_cloud->points.at(i).x;
        }
        else if (rgb_cloud->points.at(i).x < low_x)
        {
            low_x = rgb_cloud->points.at(i).x;
        }
        if (rgb_cloud->points.at(i).y > top_y)
        {
            top_y = rgb_cloud->points.at(i).y;
        }
        else if (rgb_cloud->points.at(i).y < low_y)
        {
            low_y = rgb_cloud->points.at(i).y;
        }
    }

    int costmap_row = round((top_y - low_y) / param_resolution) + 10;
    int costmap_col = round((top_x - low_x) / param_resolution) + 10;

    std::cout << "costmap size: " << costmap_row << "x" << costmap_col << std::endl;

    cv::Mat costmap(costmap_row, costmap_col, CV_8UC1, 255);

    start_point.x = 5 - round(low_x / param_resolution);
    start_point.y = 5 + round(top_y / param_resolution);

    std::cout << "start point cv_position: " << start_point << std::endl;

    std::vector<cv::Point> crub_top_vec, crub_blw_vec, slow_buf_vec;

    int max_top_x = 0;
    int min_top_x = INT_MAX;
    int max_blw_x = 0;
    int min_blw_x = INT_MAX;
    int max_buf_y = 0;
    int min_buf_y = INT_MAX;

    for (unsigned int k = 0; k < rgb_cloud->points.size(); ++k)
    {
        if (rgb_cloud->points.at(k).r == 255)
        {
            cv::Point2i this_cv_point = Point3Dto2D(rgb_cloud->points.at(k), start_point);
            costmap.at<uchar>(this_cv_point.y, this_cv_point.x) = 0;

            if ((this_cv_point.y < costmap.rows / 2) && (this_cv_point.y > 0))
            {
                if (this_cv_point.x > max_top_x)
                {
                    max_top_x = this_cv_point.x;
                }
                if (this_cv_point.x < min_top_x)
                {
                    min_top_x = this_cv_point.x;
                }

                crub_top_vec.push_back(this_cv_point);
            }
            else if ((this_cv_point.y >= costmap.rows / 2) && (this_cv_point.y < costmap.rows))
            {
                if (this_cv_point.x > max_blw_x)
                {
                    max_blw_x = this_cv_point.x;
                }
                if (this_cv_point.x < min_blw_x)
                {
                    min_blw_x = this_cv_point.x;
                }

                crub_blw_vec.push_back(this_cv_point);
            }
            else
            {
                ROS_ERROR("Error pixel value in GenMap");
                std::cout << "ErrorPointValue: " << this_cv_point << std::endl;
            }
        }
        else if (rgb_cloud->points.at(k).b == 255)
        {
            cv::Point2i this_cv_point = Point3Dto2D(rgb_cloud->points.at(k), start_point);
            costmap.at<uchar>(this_cv_point.y, this_cv_point.x) = 155;

            if (this_cv_point.y > max_buf_y)
            {
                max_buf_y = this_cv_point.y;
            }
            if (this_cv_point.y < min_buf_y)
            {
                min_buf_y = this_cv_point.y;
            }

            slow_buf_vec.push_back(this_cv_point);
        }
        else if ((rgb_cloud->points.at(k).r == 0) && (rgb_cloud->points.at(k).b == 0))
        {
            continue;
        }
        else
        {
            ROS_ERROR("Error IN Generate2DMAp");
            return FAILED;
        }
    }

    /***
     * 拟合
     */

    cv::Mat costmap_line(costmap.rows, costmap.cols, CV_8UC1, 255);

    std::pair<cv::Point2d, cv::Point2d> point_pair; //起止点
    std::vector<std::pair<cv::Point2d, cv::Point2d>> point_vec;

    // crub_blw
    cv::Vec4f crub_blw_line_param;
    cv::fitLine(crub_blw_vec, crub_blw_line_param, cv::DIST_L2, 0, 0.01, 0.01);
    //获取点斜式的点和斜率
    double crub_blw_cos_theta = crub_blw_line_param[0];
    double crub_blw_sin_theta = crub_blw_line_param[1];
    double crub_blw_x0 = crub_blw_line_param[2];
    double crub_blw_y0 = crub_blw_line_param[3];
    double crub_blw_k = crub_blw_sin_theta / crub_blw_cos_theta;
    double crub_blw_b = crub_blw_y0 - crub_blw_k * crub_blw_x0;
    double crub_blw_x1 = max_blw_x;
    double crub_blw_y1 = crub_blw_k * crub_blw_x1 + crub_blw_b;
    double crub_blw_x2 = min_blw_x;
    double crub_blw_y2 = crub_blw_k * crub_blw_x2 + crub_blw_b;
    cv::line(costmap_line, cv::Point(crub_blw_x1, crub_blw_y1), cv::Point(crub_blw_x2, crub_blw_y2), cv::Scalar(0), 2, 8);

    point_pair.second = cv::Point(crub_blw_x1, crub_blw_y1);
    point_pair.first = cv::Point(crub_blw_x2, crub_blw_y2);
    point_vec.push_back(point_pair);

    // crub top
    cv::Vec4f crub_top_param;
    cv::fitLine(crub_top_vec, crub_top_param, cv::DIST_L2, 0, 0.01, 0.01);
    //获取点斜式的点和斜率
    double crub_top_cos_theta = crub_top_param[0];
    double crub_top_sin_theta = crub_top_param[1];
    double crub_top_x0 = crub_top_param[2];
    double crub_top_y0 = crub_top_param[3];
    double crub_top_k = crub_top_sin_theta / crub_top_cos_theta;
    double crub_top_b = crub_top_y0 - crub_top_k * crub_top_x0;
    // 点的最左最右
    double crub_top_x1 = max_top_x;
    double crub_top_y1 = crub_top_k * crub_top_x1 + crub_top_b;
    double crub_top_x2 = min_top_x;
    double crub_top_y2 = crub_top_k * crub_top_x2 + crub_top_b;
    cv::line(costmap_line, cv::Point(crub_top_x1, crub_top_y1), cv::Point(crub_top_x2, crub_top_y2), cv::Scalar(0), 2, 8);

    point_pair.first = cv::Point(crub_top_x1, crub_top_y1);  //max_x
    point_pair.second = cv::Point(crub_top_x2, crub_top_y2); //min_x
    point_vec.push_back(point_pair);

    //中线
    double crub_mid_k = crub_top_k;
    double crub_mid_b = (crub_top_b + crub_blw_b) / 2.0;
    double crub_mid_x1 = max_top_x - 200;
    double crub_mid_y1 = crub_mid_k * crub_mid_x1 + crub_mid_b;
    double crub_mid_x2 = min_top_x + 200;
    double crub_mid_y2 = crub_mid_k * crub_mid_x2 + crub_mid_b;
    point_pair.first = cv::Point(crub_mid_x2, crub_mid_y2);  //min_x
    point_pair.second = cv::Point(crub_mid_x1, crub_mid_y1); //max_x
    point_vec.push_back(point_pair);
    cv::line(costmap_line, point_pair.first, point_pair.second, cv::Scalar(0), 2, 8);

    cv::line(costmap_line, cv::Point(crub_blw_x1, crub_blw_y1), cv::Point(crub_top_x1, crub_top_y1), cv::Scalar(0), 2, 8);
    cv::line(costmap_line, cv::Point(crub_blw_x2, crub_blw_y2), cv::Point(crub_top_x2, crub_top_y2), cv::Scalar(0), 2, 8);

    // slow_buf
    cv::Vec4f slow_buf_line_param;
    cv::fitLine(slow_buf_vec, slow_buf_line_param, cv::DIST_L2, 0, 0.01, 0.01);
    //获取点斜式的点和斜率
    double slow_buf_cos_theta = slow_buf_line_param[0];
    double slow_buf_sin_theta = slow_buf_line_param[1];
    double slow_buf_x0 = slow_buf_line_param[2];
    double slow_buf_y0 = slow_buf_line_param[3];
    double slow_buf_k = slow_buf_sin_theta / slow_buf_cos_theta;
    double slow_buf_b = slow_buf_y0 - slow_buf_k * slow_buf_x0;
    double slow_buf_y1 = max_buf_y;
    double slow_buf_x1 = (slow_buf_y1 - slow_buf_b) / slow_buf_k;
    double slow_buf_y2 = min_buf_y;
    double slow_buf_x2 = (slow_buf_y2 - slow_buf_b) / slow_buf_k;
    cv::line(costmap_line, cv::Point(slow_buf_x1, slow_buf_y1), cv::Point(slow_buf_x2, slow_buf_y2), cv::Scalar(50), 2, 8);

    point_pair.first = cv::Point(slow_buf_x1, slow_buf_y1);
    point_pair.second = cv::Point(slow_buf_x2, slow_buf_y2);
    point_vec.push_back(point_pair);

    BuildLaneletMap(point_vec);
    cv::imwrite(this_node_path + "/maps/costmap_origin.png", costmap);
    cv::imwrite(this_node_path + "/maps/costmap.png", costmap_line);

    // save costmap config yaml
    YAML::Node config = YAML::LoadFile(this_node_path + "/maps/costmap.yaml");
    std::ofstream config_yaml(this_node_path + "/maps/costmap.yaml");
    config["image"] = this_node_path + "/maps/costmap.yaml";
    config["resolution"] = float(param_resolution);
    config["origin_x"] = start_point.x;
    config["origin_y"] = start_point.y;
    config["negate"] = 0;
    config["occupied_thresh"] = 0.6;
    config["free_thresh"] = 0;
    config_yaml << config;

    // cv::imshow("costmap_line", costmap_line);
    // cv::waitKey(0);
    return OK;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AerialViewerPoints::TransformRGBPoints(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in, const PointTPose &trans)
{
    Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());
    this_transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisf(trans.yaw, Eigen::Vector3f::UnitZ()) *
                                             Eigen::AngleAxisf(trans.pitch, Eigen::Vector3f::UnitY()) *
                                             Eigen::AngleAxisf(trans.roll, Eigen::Vector3f::UnitX()))
                                                .toRotationMatrix();
    this_transformation(0, 3) = trans.x;
    this_transformation(1, 3) = trans.y;
    this_transformation(2, 3) = trans.z;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud_in, *tf_cloud, this_transformation);
    return tf_cloud;
}

geometry_msgs::Point AerialViewerPoints::Point2Dto3D(cv::Point cv_point)
{
    geometry_msgs::Point ros_point;
    ros_point.x = (cv_point.x - start_point.x) * param_resolution;
    ros_point.y = (start_point.y - cv_point.y) * param_resolution;
    ros_point.z = 0;

    return ros_point;
}

cv::Point2i AerialViewerPoints::Point3Dto2D(pcl::PointXYZRGB pcl_point, cv::Point start_point)
{
    return cv::Point2i(start_point.x + pcl_point.x / param_resolution, start_point.y - pcl_point.y / param_resolution);
    // return cv::Point2i(5 - (pcl_point.x/param_resolution), 5 + (pcl_point.y/param_resolution));
}

cv::Vec3b AerialViewerPoints::GetRGB(pcl::PointXYZRGB pcl_point)
{
    return cv::Vec3b(pcl_point.r, pcl_point.g, pcl_point.b);
}
