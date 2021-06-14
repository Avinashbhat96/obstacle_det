
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <cmath>
#include "geometry_msgs/Point.h"

ros::Publisher pub, pub_points;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &input){
    sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2::Ptr final_cloud (new sensor_msgs::PointCloud2);
    pcl::PCLPointCloud2* cloud_filtered_blob (new pcl::PCLPointCloud2);

    // conversion to PCLPointCloud2 type
    pcl::PCLPointCloud2* cloud (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2ConstPtr cloudptr (cloud);
    pcl_conversions::toPCL(*input, *cloud);

    // Voxel filter to downsample point cloud
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudptr);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered_blob);

    // type conversion for plane segementation task
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    // RANSAC algorithm to remove the floor from the point cloud
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int nr_points = (int) cloud_filtered->points.size();

    while(cloud_filtered->points.size() > 0.3 * nr_points){
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);

        extract.setInputCloud(cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);

        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap (cloud_f);
    }

    // Finds the minimum distance of an obstacle
    double min_x, min_y, min_z;
    double temp {4.0};
    double dist {};
    for (size_t i {0}; i < cloud_p->points.size(); i++){
        dist = sqrt(pow(cloud_p->points[i].x, 2) + pow(cloud_p->points[i].y, 2));
        if (dist < temp){
            min_x = cloud_p->points[i].x;
            min_y = cloud_p->points[i].y;
            min_z = cloud_p->points[i].z;

            temp = dist;
        }
    }

    geometry_msgs::Point points;
    points.x = min_x;
    points.y = min_y;
    points.z = min_z;

    // nearest obstacle message
    pub_points.publish(points);
    
    // type conversion to ros message
    pcl::toROSMsg(*cloud_p, *final_cloud);

    // filtered cloud, after plane segmentation
    pub.publish(final_cloud);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe ("/xtion/depth_registered/points/", 1, cloud_callback);
    pub = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
    pub_points = n.advertise<geometry_msgs::Point>("closest_obstacle", 1);

    ros::spin();
}