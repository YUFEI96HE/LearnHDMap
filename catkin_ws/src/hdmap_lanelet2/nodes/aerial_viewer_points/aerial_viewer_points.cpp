#include "aerial_viewer_points.h"

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

    nh_private.param<std::string>("calibration_file", calibration_file, "");
    nh_private.param<std::string>("divided_data_path", param_main_path, "");
    nh_private.param<std::string>("pose_6d_path", param_6d_path, "");
    nh_private.param<std::string>("image_id", param_image_id, "");
    nh_private.param<std::string>("lidar_id", param_lidar_id, "");
    nh_private.param<int>("data_num", param_data_count, 0);
    nh_private.param<int>("height", ImgPro.height, 0);
    nh_private.param<int>("width", ImgPro.width, 0);
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

    pub_fusion_cloud = nh_private.advertise<sensor_msgs::PointCloud2>("fusison_output_topic", 1);

    MainLoop();
}

void AerialViewerPoints::MainLoop()
{
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
            ImgPro.UndistortImage(current_image, current_undistorted_img);
            // cv::imshow("current_undistorted_img", current_undistorted_img);
            // cv::waitKey(300);
        }
        else
        {
            std::cout << "ERROR IN MAINLOOP" << std::endl;
        }
        ColoringPoints();
    }
    AggreateMap(param_main_path, param_6d_path);
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
    current_image_path = param_main_path + "jpg/" + std::to_string(count) + ".jpg";
    current_pcd_path = param_main_path + "pcd/" + std::to_string(count) + ".pcd";
    // json_path = param_main_path + "json/" + std::to_string(i) + ".json";

    // std::cout << current_image_path << "\n"
    //   << current_pcd_path << std::endl;

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
    return OK;
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

    // std::cout << "height: " << ImgPro.height << " "
    //         << "width: " << ImgPro.width << " "
    //         << "fx: " << ImgPro.fx << " "
    //         << "fy: " << ImgPro.fy << " "
    //         << "cx: " << ImgPro.cx << " "
    //         << "cy: " << ImgPro.cy << " "
    //         << "k1: " << ImgPro.k1 << " "
    //         << "k2: " << ImgPro.k2 << std::endl;

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
        if ((col >= 0) && (col < ImgPro.width) && (row >= 0) && (row < ImgPro.height))
        // && (current_pcd[i].z > -10))
        // if ((col >= 0) && (row >= 0) && (ImgPro.width / 2 < col) && (col < ImgPro.width) && (ImgPro.height / 2 < row) && (row < ImgPro.height))
        // && (-10 < current_pcd.points[i].z) && (current_pcd.points[i].z < 10))
        {
            // std::cout << "height: " << ImgPro.height << " " << "width: " << ImgPro.width << std::endl;
            // std::cout << "in_cloud size: " << in_cloud->points.size() << "     x: " << in_cloud->points[i].x << "   "
            //           << "y: " << in_cloud->points[i].y << "   "
            //           << "z: " << in_cloud->points[i].z << std::endl;
            // std::cout << "col: " << col << "   " << "row: " << row << std::endl;
            // ROS_WARN("GOOD");
            colored_3d_point.x = current_pcd.points[i].x;
            colored_3d_point.y = current_pcd.points[i].y;
            colored_3d_point.z = current_pcd.points[i].z;

            cv::Vec3b rgb_pixel = current_undistorted_img.at<cv::Vec3b>(row, col);

            colored_3d_point.r = rgb_pixel[2];
            colored_3d_point.g = rgb_pixel[1];
            colored_3d_point.b = rgb_pixel[0];
            // // ROS_WARN("555");
            rgb_cloud->points.push_back(colored_3d_point);
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
    std::string save_name = current_image_path + "_undistorted.pcd";
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
        // if (!((i + 1) % intervel == 0))
        // {
        //     continue;
        // }

        tmp_pcd_path = rgb_pcd_file_path + "jpg/" + std::to_string(i) + ".jpg.pcd";
        std::cout << "rgb_pcd_path: " << tmp_pcd_path << std::endl;

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

    sensor_msgs::PointCloud2 out_cloud_msg;
    pcl::toROSMsg(*save_map, out_cloud_msg);
    out_cloud_msg.header.frame_id = param_lidar_id;
    out_cloud_msg.header.stamp = ros::Time();
    pub_fusion_cloud.publish(out_cloud_msg);

    // PCDViewer(save_path);
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