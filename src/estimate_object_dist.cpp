// PCLを使用して特定の矩形内の深度データの中央値を計算し、ROScppで出力する
// 矩形の範囲は(x_min, y_min)から(x_max, y_max)の範囲で指定
//


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/organized_edge_detection.h>

ros::Publisher depth_median_pub;
double x_min, x_max, y_min, y_max;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::CropBox<pcl::PointXYZRGB> cropFilter;
    cropFilter.setInputCloud(cloud);
    Eigen::Vector4f minPoint(x_min, y_min, 0, 1);
    Eigen::Vector4f maxPoint(x_max, y_max, 10, 1); // 10 is an arbitrary large value for z max
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cropFilter.filter(*cropped_cloud);

    std::vector<float> depths;
    for (const auto& point : cropped_cloud->points) {
        depths.push_back(point.z);
    }

    // Calculate median depth
    size_t n = depths.size() / 2;
    std::nth_element(depths.begin(), depths.begin() + n, depths.end());
    float median_depth = depths[n];

    ROS_INFO("Median Depth: %f", median_depth);

    // Publish median depth
    // (Assuming you have initialized depth_median_pub in the main function)
    // depth_median_pub.publish(median_depth);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_median_calculation");
    ros::NodeHandle nh;

    nh.getParam("/depth_rect/x_min", x_min);
    nh.getParam("/depth_rect/x_max", x_max);
    nh.getParam("/depth_rect/y_min", y_min);
    nh.getParam("/depth_rect/y_max", y_max);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, cloudCallback);
    depth_median_pub = nh.advertise<float>("depth_median", 1);

    ros::spin();

    return 0;
}

