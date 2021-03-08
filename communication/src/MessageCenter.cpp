/*
 * @file: 
 * 
 * @brief: 
 * 
 * @author: heyufei
 * 
 * @data: 2020-11-12
 * 
 */

#include "MessageCenter.h"

unsigned int now_client_num = 0;
pthread_spinlock_t now_client_num_lock;

MessageCenter::MessageCenter(/* args */)
{
}
MessageCenter::~MessageCenter()
{
}

void MessageCenter::CallbackGetIMU(const sensor_msgs::Imu::ConstPtr &msg_imu)
{
    imu_data_linear_acceleration_x = msg_imu->linear_acceleration.x;
    imu_data_linear_acceleration_y = msg_imu->linear_acceleration.y;
    imu_data_linear_acceleration_z = msg_imu->linear_acceleration.z;
    imu_data_angular_velocity_x = msg_imu->angular_velocity.x;
    imu_data_angular_velocity_y = msg_imu->angular_velocity.y;
    imu_data_angular_velocity_z = msg_imu->angular_velocity.z;
}
void MessageCenter::CallbackGetOdom(const nav_msgs::Odometry::ConstPtr &msg_odom)
{
    position_x = msg_odom->pose.pose.position.x;
    position_y = msg_odom->pose.pose.position.y;
    position_z = msg_odom->pose.pose.position.z;
    quaternion_w = msg_odom->pose.pose.orientation.w;
    quaternion_x = msg_odom->pose.pose.orientation.x;
    quaternion_y = msg_odom->pose.pose.orientation.y;
    quaternion_z = msg_odom->pose.pose.orientation.z;
    angle = (float)(atan2(2 * (quaternion_x * quaternion_y + quaternion_w * quaternion_z), quaternion_w * quaternion_w + quaternion_x * quaternion_x - quaternion_y * quaternion_y - quaternion_z * quaternion_z) * 57.3);
}

void MessageCenter::PubUltrasound()
{
    std_msgs::Int64MultiArray ultra_msg;
    for (int i = 0; i < 8; ++i)
    {
        ultra_msg.data.push_back(han_ccu->ultrasound[i]);
    }
    if (ultra_msg.data.size() == 8)
    {
        ultrasonic_pub.publish(ultra_msg);
    }
}
void MessageCenter::PubDirection()
{
    std_msgs::Int64MultiArray forward_backward_msg;
    forward_backward_msg.data.push_back(han_ccu->forward_backward);
    forward_backward_pub.publish(forward_backward_msg);
}
void MessageCenter::PubVolocity()
{
    geometry_msgs::TwistStamped volocity_msg;
    volocity_msg.header.stamp = ros::Time::now();
    volocity_msg.twist.linear.x = han_ccu->wheel_speed;
    wheel_speed_pub.publish(volocity_msg);
}
/* todo */
void MessageCenter::PubTask()
{
    task_pub.publish(handle_adt->task_seq);
}

void MessageCenter::MainLoop()
{
    ros::NodeHandle nh_;
    ros::Rate loop_rate(10);
    ros::Subscriber sub_imu = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 50, &MessageCenter::CallbackGetIMU, this);
    ros::Subscriber sub_odom = nh_.subscribe<nav_msgs::Odometry>("/odom/imu", 40, &MessageCenter::CallbackGetOdom, this);

    ultrasonic_pub = nh_.advertise<std_msgs::Int64MultiArray>("ultrasonic_dis", 10);
    forward_backward_pub = nh_.advertise<std_msgs::Int64MultiArray>("going_direction", 2);
    wheel_speed_pub = nh_.advertise<geometry_msgs::TwistStamped>("wheel_circles", 10);
    task_pub = nh_.advertise<adam_msgs::VehicleCmd>("gpn_task", 1000);
    /* heyufeidebug */
    node_pub = nh_.advertise<adam_msgs::RoboNodeStatus>("app_cmd", 1000);

    while (ros::ok())
    {
        ros::spinOnce();
        PubUltrasound();
        PubDirection();
        PubVolocity();
        PubTask();
        loop_rate.sleep();
    }

    printf("Thread Error: Message Center\n");
}
