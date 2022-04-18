#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
//#include <sensor_msgs/PointCloud2.h>

struct Accelerometer {
    float ax;
    float ay;
    float az;
} accel_values;

geometry_msgs::Quaternion orientation;

ros::Publisher pub_imu;

void cbAccel(const sensor_msgs::ImuConstPtr &data)
{
    accel_values.ax = data->linear_acceleration.x;
    accel_values.ay = data->linear_acceleration.y;
    accel_values.az = data->linear_acceleration.z;
}

void cbGyro(const sensor_msgs::ImuConstPtr &data)
{
    sensor_msgs::Imu msg;
    msg = *data;
    msg.linear_acceleration.x = accel_values.ax;
    msg.linear_acceleration.y = accel_values.ay;
    msg.linear_acceleration.z = accel_values.az;
    msg.orientation = orientation;
    pub_imu.publish(msg);
}

void cbOdom(const nav_msgs::OdometryConstPtr &data)
{
    orientation = data->pose.pose.orientation;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_lio_sam");
    ros::NodeHandle n_public;

    ros::Subscriber sub_accel = n_public.subscribe("/tracking_camera/accel/sample", 10, cbAccel);
    ros::Subscriber sub_gyro = n_public.subscribe("/tracking_camera/gyro/sample", 10, cbGyro);
    ros::Subscriber sub_odom = n_public.subscribe("/tracking_camera/odom/sample", 10, cbOdom);
    pub_imu = n_public.advertise<sensor_msgs::Imu>("/imu_raw", 10);
    ros::spin();
}
