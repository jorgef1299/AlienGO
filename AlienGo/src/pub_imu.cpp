#include "pub_imu.h"
#include <sensor_msgs/PointCloud2.h>
#include "nav_msgs/Odometry.h"
#include "livox_ros_driver/CustomMsg.h"

#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl-1.8/pcl/impl/point_types.hpp>
#include "pcl-1.8/pcl/point_cloud.h"
#include "pcl-1.8/pcl/point_types.h"
#include "pcl-1.8/pcl/pcl_macros.h"
#include "pcl-1.8/pcl/io/pcd_io.h"

sensor_msgs::Imu accel_data;
ros::Publisher pub_imu;
ros::Publisher pub_point_cloud;

sensor_msgs::Imu msg_imu;
float last_linear_acceleration_x = 0;
float last_linear_acceleration_y = 0;

struct PointXYZIRT
{
    float x, y, z;
    float intensity;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
(float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
(uint16_t, ring, ring) (float, time, time)
)

pcl::PointCloud<PointXYZIRT>::Ptr cloud_out(new pcl::PointCloud<PointXYZIRT>());


void cbAccel(const sensor_msgs::ImuConstPtr &data)
{
    accel_data = *data;
}

void cbGyro(const nav_msgs::OdometryConstPtr &data)
{
    sensor_msgs::Imu msg;
    msg.header.stamp=data->header.stamp;
    msg.orientation = msg_imu.orientation;
    msg.linear_acceleration.x = accel_data.linear_acceleration.z;
    msg.linear_acceleration.y = accel_data.linear_acceleration.x;
    msg.linear_acceleration.z = accel_data.linear_acceleration.y;
    msg.angular_velocity.x = data->twist.twist.angular.x;
    msg.angular_velocity.y = data->twist.twist.angular.y;
    msg.angular_velocity.z = data->twist.twist.angular.z;
    pub_imu.publish(msg);
}

void cbImu(const sensor_msgs::ImuConstPtr &data)
{
    msg_imu = *data;
    sensor_msgs::Imu msg;
    msg = *data;
   /* if(msg.header.stamp.nsec < 749242306) {
        msg.header.stamp.nsec = 1000000000 - 749242306 + msg.header.stamp.nsec;
        msg.header.stamp.sec--;
    }
    else {
        msg.header.stamp.nsec = msg.header.stamp.nsec - 749242306;
    }
    msg.header.stamp.sec = msg.header.stamp.sec - 6486;*/
    msg.orientation.x = 0;
    msg.orientation.y = 0;
    msg.orientation.z = 0;
    msg.orientation.w = 0;
    /*msg.linear_acceleration.x = data->linear_acceleration.x;
    msg.linear_acceleration.y = data->linear_acceleration.y;
    msg.linear_acceleration.z = data->linear_acceleration.z;*/
    //if(msg.linear_acceleration.x != last_linear_acceleration_x && msg.linear_acceleration.y != last_linear_acceleration_y)
    pub_imu.publish(msg);
    last_linear_acceleration_x = msg.linear_acceleration.x;
    last_linear_acceleration_y = msg.linear_acceleration.y;
}

void cbLivoxLidarPoints(const livox_ros_driver::CustomMsgConstPtr &livox_data)
{
    /*for(uint32_t i=0; i < livox_data->point_num; i++) {
        PointXYZIRT p;
        p.x = livox_data->points[i].x;
        p.y = livox_data->points[i].y;
        p.z = livox_data->points[i].z;
        p.intensity = livox_data->points[i].reflectivity;
        p.ring = livox_data->points[i].line;
        p.time = livox_data->points[i].offset_time / 1000000000.0;
        cloud_out->push_back(p);
    }
    sensor_msgs::PointCloud2 cloud_temp;
    pcl::toROSMsg(*cloud_out, cloud_temp);
    cloud_temp.header = livox_data->header;
    pub_point_cloud.publish(cloud_temp);
    cloud_out->clear();*/
    livox_ros_driver::CustomMsg msg;
    msg = *livox_data;
    msg.header.stamp = ros::Time::now();
    pub_point_cloud.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_imu_node");
    ros::NodeHandle n_public;
    //ros::Subscriber sub_accel = n_public.subscribe("/tracking_camera/accel/sample", 1000, cbAccel);
    //ros::Subscriber sub_gyro = n_public.subscribe("/tracking_camera/odom/sample", 1000, cbGyro);
    ros::Subscriber sub_imu = n_public.subscribe("/Aliengo/imu", 1000, cbImu);
    //ros::Subscriber sub_livox_point_cloud = n_public.subscribe("livox/lidar_aux", 1, cbLivoxLidarPoints);
    pub_imu = n_public.advertise<sensor_msgs::Imu>("/livox/imu", 1000);
    //pub_point_cloud = n_public.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 1);
    ros::spin();
    return 0;
}
