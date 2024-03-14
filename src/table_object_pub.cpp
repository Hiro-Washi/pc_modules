<<<<<<< HEAD

// 目の前のテーブル上の点群を抽出（平面検出、平面の上の点群の抽出）して、
// その点群データをパブリッシュ(か保持）する.
// 
// 一番左の物体の3次元位置から

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>


class TableObjectExtractor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher segmented_cloud_pub_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorless_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_objects_cloud_;

    // 点群からRealsenseカメラの2次元画像の位置（縦の画素位置、横の画素位置）に変換する関数
    // 
public:
    PointCloudProcessing() : nh_("~") {
        // 

        // Subscribe to point cloud topic
        cloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &TableObjectExtractor::cloudCallback, this);

        // Advertise segmented point cloud topic
        //segmented_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
    }

    void downsamplePointCloud() {
        // Downsample the cloud
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
        voxel_grid_filter.setInputCloud(colorless_cloud_);
        voxel_grid_filter.setLeafSize(0.01, 0.01, 0.01); // Adjust leaf size as needed
        voxel_grid_filter.filter(*downsampled_cloud_);
    }

    void extractTablePlane() {
        // Perform plane segmentation to extract the table plane
        // This is just a placeholder, you should replace it with your actual plane segmentation code
        // Here, we assume that the table plane is extracted and stored in table_cloud_
        *table_cloud_ = *downsampled_cloud_; // Placeholder for the actual segmentation result
    }

    void extractObjectsOnTable() {
        // Perform clustering to extract objects on the table
        // This is just a placeholder, you should replace it with your actual clustering code
        // Here, we assume that the objects on the table are extracted and stored in objects_cloud_
        *objects_cloud_ = *downsampled_cloud_; // Placeholder for the actual clustering result
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_processing_node");

    TableObjectPCDetector pc_processor;

    ros::spin();

    return 0;
}



ros::Publisher table_point_cloud_pub;

// 
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*cloud_msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*pcl_cloud, *cloud);

    // Downsample the cloud to speed up processing
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.setLeafSize(0.01, 0.01, 0.01); // Adjust leaf size as needed
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid_filter.filter(*downsampled_cloud);

    // Perform plane segmentation to extract the table plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setInputCloud(downsampled_cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01); // Adjust distance threshold as needed
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        ROS_WARN("Could not find the table plane.");
        return;
    }

    // Extract the table points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(downsampled_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*table_cloud);

    // Publish the table point cloud
    sensor_msgs::PointCloud2 table_cloud_msg;
    pcl::toROSMsg(*table_cloud, table_cloud_msg);
    table_cloud_msg.header = cloud_msg->header;
    table_point_cloud_pub.publish(table_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "table_extraction_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, cloudCallback);
    table_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("table_point_cloud", 1);

    ros::spin();

    return 0;
}
