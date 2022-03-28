#include "pub_imu.h"

sensor_msgs::Imu accel_data;
ros::Publisher pub_imu;
/*
void cbAccel(const sensor_msgs::ImuConstPtr &data)
{
    accel_data = *data;
}

void cbGyro(const sensor_msgs::ImuConstPtr &data)
{
    sensor_msgs::Imu msg;
    msg = *data;
    msg.linear_acceleration.x = -accel_data.linear_acceleration.y/9.8;
    msg.linear_acceleration.y = accel_data.linear_acceleration.z/9.8;
    msg.linear_acceleration.z = accel_data.linear_acceleration.x/9.8;
    msg.angular_velocity.x = -data->angular_velocity.y;
    msg.angular_velocity.y = data->angular_velocity.z;
    msg.angular_velocity.z = data->angular_velocity.x;
    pub_imu.publish(msg);
}
*/
void cbImu(const sensor_msgs::ImuConstPtr &data)
{
    sensor_msgs::Imu msg;
    msg = *data;
    msg.linear_acceleration.x = data->linear_acceleration.x/9.8;
    msg.linear_acceleration.y = data->linear_acceleration.y/9.8;
    msg.linear_acceleration.z = data->linear_acceleration.z/9.8;
    pub_imu.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_imu_node");
    ros::NodeHandle n_public;
    //ros::Subscriber sub_accel = n_public.subscribe("/tracking_camera/accel/sample", 10, cbAccel);
    //ros::Subscriber sub_gyro = n_public.subscribe("/tracking_camera/gyro/sample", 10, cbGyro);
    ros::Subscriber sub_imu = n_public.subscribe("/Aliengo/imu", 10, cbImu);
    pub_imu = n_public.advertise<sensor_msgs::Imu>("/livox/imu", 10);
    ros::spin();
}
