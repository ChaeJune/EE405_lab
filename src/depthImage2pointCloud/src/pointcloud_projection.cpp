#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include<pcl/filters/voxel_grid.h>

using namespace std;
using namespace ros;

class PointCloudProjection{
    public:
    PointCloudProjection();
    ~PointCloudProjection();

    void DepthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void CameraInfoCllback(const sensor_msgs::CameraInfo& msg);

    pcl::PointCloud<pcl::PointXYZ> GetPointCloud(cv::Mat depth_pic);

    private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_depth_image, sub_camera_info;
    ros::Publisher pub_pointcloud, pub_pointcloudfiltered;

    string depth_raw_image, depth_image_info;
    bool info_in;
    double camera_factor, camera_cx, camera_cy, camera_fx, camera_fy;
};

PointCloudProjection::PointCloudProjection():nh_(""), private_nh_("~")
{
    private_nh_.getParam("depth_raw_image",     depth_raw_image);
    private_nh_.getParam("depth_image_info",    depth_image_info);
    private_nh_.getParam("camera_factor",       camera_factor);

    sub_depth_image = nh_.subscribe(depth_raw_image, 1, &PointCloudProjection::DepthImageCallback, this);
    sub_camera_info = nh_.subscribe(depth_image_info, 1, &PointCloudProjection::CameraInfoCllback, this);
    pub_pointcloud = nh_.advertise<sensor_msgs::PointCloud2>(depth_raw_image + "/projected_points", 10);
    pub_pointcloudfiltered = nh_.advertise<sensor_msgs::PointCloud2>(depth_raw_image + "/filtered_points", 10);

    info_in = false;

    ROS_INFO("PointCloudProjection is created");
}
PointCloudProjection::~PointCloudProjection()
{
    ROS_INFO("PointCloudProjection is distructed");
}

void PointCloudProjection::DepthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat depth_pic;
    cv_bridge::CvImagePtr depth_ptr;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudf (new pcl::PointCloud<pcl::PointXYZ>);


    try
    {
        depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); 
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }
    depth_pic = depth_ptr->image;
    *pointcloud = GetPointCloud(depth_pic);

    // Create a pass through filter to remove points too far away
    //pcl::PassThrough<pcl::PointXYZ> pass;
    //pass.setInputCloud(pointcloud);


 //   pcl::VoxelGrid<pcl::PointXYZ> fi;
 //   fi.setInputCloud(pointcloudf)
//    fi.setLeafSize(0.01f, 0.01f, 0.01f);
/*
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(pointcloud);
    voxel_grid.setLeafSize(0.01f,0.01f,0.01f);
    voxel_grid.filter(*filtered_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(pointcloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02);  // maximum distance between two points to be considered as part of the same cluster
    ec.setMinClusterSize(100);     // minimum number of points that a cluster needs to have to be considered a valid cluster
    ec.setMaxClusterSize(2500);   // maximum number of points that a cluster can have
    ec.setSearchMethod(tree);
    ec.setInputCloud(pointcloud);
    ec.extract(cluster_indices);
*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(pointcloud);
    voxel_grid.setLeafSize(0.08f,0.08f,0.08f);
    voxel_grid.filter(*filtered_cloud);
    //
    //
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    
    pass.setInputCloud(filtered_cloud);
    pass.setFilterLimits(-0.5,0.5);
    pass.setFilterLimitsNegative(true);
    pass.filter(*filtered_cloud);
    


    //rviz
    sensor_msgs::PointCloud2 detected_pointcloud_m;
    pcl::toROSMsg(*pointcloud, detected_pointcloud_m);
    detected_pointcloud_m.header.frame_id = msg->header.frame_id;
    pub_pointcloud.publish(detected_pointcloud_m);

    sensor_msgs::PointCloud2 filtered_pointcloud_m;
    pcl::toROSMsg(*filtered_cloud, filtered_pointcloud_m);
    filtered_pointcloud_m.header.frame_id = msg->header.frame_id;
    pub_pointcloudfiltered.publish(filtered_pointcloud_m);

    pointcloud = nullptr;
    filtered_cloud = nullptr;
}

void PointCloudProjection::CameraInfoCllback(const sensor_msgs::CameraInfo& msg)
{
    camera_cx = msg.K[2];
    camera_cy = msg.K[5];
    camera_fx = msg.K[0];
    camera_fy = msg.K[4];
    info_in = true;
}

pcl::PointCloud<pcl::PointXYZ> PointCloudProjection::GetPointCloud(cv::Mat depth_pic){

    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Traverse the depth image 
    for (int v = 0; v < depth_pic.rows; ++v)
    {
        for (int u = 0; u < depth_pic.cols; ++u)
        {
            float d = depth_pic.ptr<float>(v)[u];

            // Check for invalid measurements
            if (d == 0)
                continue;
            // if d has a value, add a point to the point cloud 

            pcl::PointXYZ pt;

            // Fill in XYZ
            pt.z = double(d) / camera_factor;
            pt.x = (u - camera_cx) * pt.z / camera_fx;
            pt.y = (v - camera_cy) * pt.z / camera_fy;

            // add p to the point cloud 
            cloud.points.push_back(pt);
        }
    }

    cloud.height   = 1;
    cloud.width    = cloud.points.size();
    cloud.is_dense = false;

    return cloud;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PointCloudProjection_node");
    PointCloudProjection detection_2_pointcloud;
    
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
