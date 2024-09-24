#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/tf.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> // 用于转换ROS PointCloud2到PCL点云格式


// IMU 回调函数
void IMUCallback(const sensor_msgs::Imu msg)
{
    // 检测消息包中四元数数据是否存在
    if(msg.orientation_covariance[0] < 0)
        return;
    // 四元数转成欧拉角
    tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // 弧度换算成角度
    roll = roll*180/M_PI;
    pitch = pitch*180/M_PI;
    yaw = yaw*180/M_PI;
    ROS_INFO("滚转= %.000f 俯仰= %.000f 朝向= %.000f", roll, pitch, yaw);
}

void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // 将ROS PointCloud2消息转换为PCL点云格式
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // 输出点云中的点数
    ROS_INFO("Received PointCloud2 message. Number of points: %ld",cloud->size());
    // std::cout << "Received PointCloud2 message. Number of points: " << cloud->size() << std::endl;
}


int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc,argv, "demo_imu_data"); 

    ros::NodeHandle n;
    // 订阅 IMU 的数据话题
    ros::Subscriber sub_imu = n.subscribe("/fd_imu/imu", 50, IMUCallback);
    ros::Subscriber sub_imu2 = n.subscribe("/imu_raw", 50, IMUCallback);
    ros::Subscriber sub_lidar = n.subscribe("/points_raw", 10, LidarCallback);
    ros::spin();

    return 0;
}
