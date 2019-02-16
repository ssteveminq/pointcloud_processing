#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>

// Approximate time policy
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

// Darknet detection
#include <darknet_ros_msgs/BoundingBoxes.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_cloud, const darknet_ros_msgs::BoundingBoxesConstPtr& input_detection)
{

  // Initialize containers for clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg( *input_cloud, *cloud);
  pcl::PointCloud<pcl::PointXYZRGB> cloud_output;
  
  int num_boxes = input_detection->bounding_boxes.size();
  int width = input_cloud->width;
  int height = input_cloud->height;

  
  for(int i(0); i<num_boxes; i++){
    
    // Unwrap darknet detection
    std::string name = input_detection->bounding_boxes[i].Class;
    int xmin  = input_detection->bounding_boxes[i].xmin;
    int xmax  = input_detection->bounding_boxes[i].xmax;
    int ymin  = input_detection->bounding_boxes[i].ymin;
    int ymax  = input_detection->bounding_boxes[i].ymax;

    // Get inliers
    std::vector<int> indices;

    for (int column(xmin); column<=xmax; column++){
      for (int row(ymin); row<=ymax; row++){
        indices.push_back(row*width+column);
      }
    }

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    inliers->indices = indices;

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_filtered);
    cloud_output += *cloud_filtered;
    

  }

  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Convert pcl cloud to ROS message
  pcl::toROSMsg(cloud_output, output);

  // Output cloud has the same frame_id as the input
  output.header.frame_id=input_cloud->header.frame_id;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "roi_extractor");
  ros::NodeHandle nh;

  // Initialize subscribers to darknet detection and pointcloud
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud(nh, "hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_box(nh, "darknet_ros/bounding_boxes", 1);

  // Synchronize darknet detection with pointcloud
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_cloud, sub_box);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("hsrb/head_rgbd_sensor/depth_registered/rectified_points/ROI", 1);

  // Spin
  ros::spin ();
}
