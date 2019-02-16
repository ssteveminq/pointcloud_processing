#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

ros::Publisher pub;
ros::Publisher pub2;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  ROS_INFO("Pointcloud received"); 

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg (*input, *cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.015);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  // Create filtered cloud container
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered  (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create output container
  pcl::PointCloud<pcl::PointXYZRGB> cloud_out;  

  // Create auxiliary cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aux  (new pcl::PointCloud<pcl::PointXYZRGB>);

  int i = 0, nr_points = (int) cloud->points.size ();
  // While 60% of the original cloud is still there
  while (cloud->points.size () > 0.6 * nr_points)
  {

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      ROS_INFO("Could not estimate a planar model for the given dataset.");
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_filtered);

    // Add points to pointcloud
    cloud_out += *cloud_filtered;
    
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_aux);
    cloud.swap (cloud_aux);

  }

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 output2;
  pcl::toROSMsg(cloud_out, output);
  pcl::toROSMsg(*cloud, output2);

  // Set output frame as the input frame
  output.header.frame_id=(*input).header.frame_id;
  output2.header.frame_id=(*input).header.frame_id;

  // Publish output
  pub.publish (output);
  pub2.publish (output2);

  ROS_INFO("Pointcloud published");
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "planar_segmentation");
  ros::NodeHandle nh;

  ROS_INFO("Pointcloud processing started"); 

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("hsrb/head_rgbd_sensor/depth_registered/rectified_points/plane_segmentation_in", 1);

  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("hsrb/head_rgbd_sensor/depth_registered/rectified_points/plane_segmentation_out", 1);

  // Spin
  ros::spin ();
}
