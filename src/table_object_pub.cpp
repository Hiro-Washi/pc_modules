#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Convert ROS PointCloud2 message to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    // Filter out points with depth greater than 1.5m
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& point : pcl_cloud->points)
    {
        if (point.z <= 1.5 && point.z >= 0)
        {
            filtered_cloud->points.push_back(point);
        }
    }

    // Plane segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setInputCloud(filtered_cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.segment(*inliers, *coefficients);

    // Output plane coefficients
    ROS_INFO("Plane coefficients: [%f, %f, %f, %f]", coefficients->values[0], coefficients->values[1],
             coefficients->values[2], coefficients->values[3]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/point_cloud", 1, cloudCallback);

    ros::spin();

    return 0;
}
