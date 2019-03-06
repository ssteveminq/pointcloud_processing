#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <pointcloud_processing_msgs/handle_position.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h> 

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

const float SURVIVAL_TIME = 60.0;
const float TEMP_OCC_TIME = 2.0;
const int QUEUE_SIZE = 30;

struct ObjInfo{
    float x;
    float y;
    int width;
    int height;
    std::string label;
    geometry_msgs::PointStamped center;
    ros::Time last_time;
    float average_depth;
    bool no_observation;
};



// Initialize publishers
ros::Publisher pub_bottle;
ros::Publisher pub_bottle_centroid;
ros::Publisher pub_bottle_poses;
ros::Publisher pub_cup;
ros::Publisher pub_cup_centroid;
ros::Publisher pub_cup_poses;
ros::Publisher pub_handle_centroid_list;

// Initialize transform listener
tf::TransformListener* lst;

// Set fixed reference frame
std::string fixed_frame = "map";

//map
std::map<std::string, ObjInfo> labels_to_obj;
std::vector<std::string> target_string;
std::map<std::string, std::deque<float>> depth_buffer;


//If _name is in target_strings, return true
bool Is_target(std::string _name)
{
    bool Is_target = false;
    for(size_t i(0);i< target_string.size();i++)
    {
        if(target_string[i].compare(_name)==0)
        {
            return true;
        }
    }
    return false;
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_cloud, const darknet_ros_msgs::BoundingBoxesConstPtr& input_detection)
{
  //ROS_INFO("cloud callback");

  // Initialize containers for clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud                  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bottle           (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cup      (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Handle orientation
  //std::vector<std::string> handle_orientation;

  // Initialize container for object poses
  geometry_msgs::PoseArray bottle_poses,cup_poses;
  bottle_poses.header.stamp = cup_poses.header.stamp = ros::Time::now();
  bottle_poses.header.frame_id = cup_poses.header.frame_id = fixed_frame;

  // Initialize container for centroids' markers
  visualization_msgs::Marker centroid_bottle_list, centroid_cup_list ;

  centroid_bottle_list.header.frame_id = centroid_cup_list.header.frame_id = fixed_frame;

   centroid_bottle_list.type = centroid_cup_list.type = 7;

  centroid_bottle_list.color.a = centroid_cup_list.color.a = 1.0;

  centroid_bottle_list.color.r = centroid_cup_list.color.r = 1.0;

  centroid_bottle_list.action = centroid_cup_list.action = 0;

  centroid_bottle_list.scale.x = centroid_cup_list.scale.x = 0.05;

  centroid_bottle_list.scale.y = centroid_cup_list.scale.y = 0.05;

  centroid_bottle_list.scale.z = centroid_cup_list.scale.z = 0.05;

  // Initialize message with handle detections
  pointcloud_processing_msgs::handle_position handle_list;

  // Initialize container for auxiliary clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_roi            (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_voxelgrid      (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_sor            (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_inplane        (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_outplane       (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::fromROSMsg( *input_cloud, *cloud);

  // Get cloud width and height
  int width = input_cloud->width;
  int height = input_cloud->height;

  // Number of objects detected
  int num_boxes = input_detection->bounding_boxes.size();

  // For each object detected
  for(int i(0); i<num_boxes; i++){
    
    // Unwrap darknet detection
    //ros::Time secs =ros::Time::now();
    std::string object_name = input_detection->bounding_boxes[i].Class;
    int xmin                = input_detection->bounding_boxes[i].xmin;
    int xmax                = input_detection->bounding_boxes[i].xmax;
    int ymin                = input_detection->bounding_boxes[i].ymin;
    int ymax                = input_detection->bounding_boxes[i].ymax;
    float probability = input_detection->bounding_boxes[i].probability;
    if(probability<0.6)
        return;

    // ---------------Determine handle orientation--------------------
    //if (object_name=="bottle"){
      //if ((xmax-xmin)>=(ymax-ymin)){
        //handle_orientation.push_back("horizontal");
      //}
      //else{
        //handle_orientation.push_back("vertical");
      //}
    //}

    // -------------------ROI extraction------------------------------

    // Initialize container for inliers of the ROI
    pcl::PointIndices::Ptr inliers_roi (new pcl::PointIndices ());

    // Get inliers
    std::vector<int> indices;
    for (int column(xmin); column<=xmax; column++){
      for (int row(ymin); row<=ymax; row++){
        // Pixel coordinates to pointcloud index
        indices.push_back(row*width+column);
      }
    }

    inliers_roi->indices = indices;

    // Create the filtering ROI object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_roi;
    // Extract the inliers  of the ROI
    extract_roi.setInputCloud (cloud);
    extract_roi.setIndices (inliers_roi);
    extract_roi.setNegative (false);
    extract_roi.filter (*cloud_filtered_roi);
    
    /*
    // ----------------------VoxelGrid----------------------------------
    // Perform the downsampling
    pcl::VoxelGrid<pcl::PointXYZRGB> sor_voxelgrid;
    sor_voxelgrid.setInputCloud (cloud_filtered_roi);
    sor_voxelgrid.setLeafSize (0.01, 0.01, 0.01); //size of the grid
    sor_voxelgrid.filter (*cloud_filtered_voxelgrid);

   // Exception
   if (cloud_filtered_voxelgrid->points.size() < 3){
     if (object_name=="bottle"){handle_orientation.pop_back();}
   break;
   }
   */

   // ---------------------StatisticalOutlierRemoval--------------------

   // Create the filtering object
   pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_noise;
   // Remove noise
   sor_noise.setInputCloud (cloud_filtered_roi);
   //sor_noise.setInputCloud (cloud_filtered_voxelgrid);
   sor_noise.setMeanK (50);
   sor_noise.setStddevMulThresh (1.0);
   sor_noise.filter (*cloud_filtered_sor);


   //remove NaN points from the cloud
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr nanfiltered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   std::vector<int> rindices;
   pcl::removeNaNFromPointCloud(*cloud_filtered_sor,*nanfiltered_cloud, rindices);

   float avg_depth =0;
   float nearest=0.0;
   float depth_maxcount=0.0;
   int depth_count =nanfiltered_cloud->points.size();
   //std::cout<<"size : "<<depth_count <<std::endl;
   //std::vector<float> depthvector;
   //depthvector.resize(depth_count);
   for(size_t i=0;i<nanfiltered_cloud->points.size();i++)
   {
       //std::cout<<"inside clouds"<<nanfiltered_cloud->points[i].z<<std::endl;
       float cur_depth = nanfiltered_cloud->points[i].z;
       avg_depth+=nanfiltered_cloud->points[i].z;
       nearest =roundf(cur_depth*1000) /1000.0;
       //depthvector[i]=nearest;
   }

   //If depth value is not reliable, don't use this value to update target object information
   if(depth_count>0)
       avg_depth =avg_depth/depth_count;
   else
       continue;
    

   if(Is_target(object_name) && !std::isnan(avg_depth) )
   {
       auto it_depthvector = depth_buffer.find(object_name);
       if(it_depthvector !=depth_buffer.end() )
           if(it_depthvector->second.size()>QUEUE_SIZE)
           {
               it_depthvector->second.push_back(avg_depth);
               it_depthvector->second.pop_front();
           }
           else
               it_depthvector->second.push_back(avg_depth);
   }

   if(Is_target(object_name) && avg_depth ==0.0)
   {
       auto it_depthvector = depth_buffer.find(object_name);
        if(it_depthvector !=depth_buffer.end() )
        {
           if(it_depthvector->second.size()>0)
           {
               float avg_buffer=0.0;
               int   avg_count=0;
               avg_depth =0.0;
               for(size_t j(0);j<it_depthvector->second.size();j++)
               {
                   if(std::isnan(it_depthvector->second[j]) && (it_depthvector->second[j]!=0.0))
                   {
                       avg_buffer+=it_depthvector->second[j];
                       avg_count++;
                   }
               }
               if(avg_count>0)
                   avg_buffer = avg_buffer/avg_count;

               avg_depth= avg_buffer ;
               //avg_depth =(it_depthvector->second[it_depthvector->second.size()-10]);
               ROS_INFO("%s depath is nan, so, put into deque value: %.3f",object_name.c_str(), avg_buffer);

           }
            else
            {
               avg_depth = 0.0;
               ROS_INFO("%s depath is nan, so, put into Zero----last casee: %.3f",object_name.c_str(), avg_depth);
            }
        }
   }

   /*
   int co=0;
    int nmax=0;
    //fid the most common value in depth vector
    if(depthvector.size()>0)
    {
        float mostvalue=depthvector[0];
        for(int i=0;i<depthvector.size();i++)
        {
            co = (int) std::count(depthvector.begin(), depthvector.end(), depthvector[i]);
            if(co > nmax)
            {       nmax = co;
                mostvalue = depthvector[i];
            }
        }

    //ROS_INFO("avg : %.3lf, most_common: %.3lf ", depth, mostvalue);
    depth_maxcount = mostvalue;
    }
    */

   ROS_INFO("object_name : %s , depth_avg: %.3lf ",object_name.c_str(),  avg_depth );


   /*
   // ---------------------RANSAC_PlanarSegmentation--------------------

   // Initialize containers for plane coefficients and inliers of the plane
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
   pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  // Segment the largest planar component from the remaining cloud
  seg.setInputCloud (cloud_filtered_sor);
  seg.segment (*inliers_plane, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_plane;

  // Extract the inliers
  extract_plane.setInputCloud (cloud_filtered_sor);
  extract_plane.setIndices (inliers_plane);
  extract_plane.setNegative (false);
  extract_plane.filter (*cloud_filtered_inplane);
    
  // Extract the outliers

  extract_plane.setNegative (true);
  extract_plane.filter (*cloud_filtered_outplane);


  // ----------------------Compute normal vector-----------------------

  // Initialize container for vectors
  geometry_msgs::Vector3Stamped normal_vector_rel;
  geometry_msgs::Vector3Stamped normal_vector_abs;

  normal_vector_rel.header.stamp = ros::Time::now();
  normal_vector_rel.header.frame_id = coefficients->header.frame_id;
  normal_vector_rel.vector.x = coefficients->values[0];
  normal_vector_rel.vector.y = coefficients->values[1];
  normal_vector_rel.vector.z = coefficients->values[2];

  // make sure the normal vector is pointing inwards the object

  if (normal_vector_rel.vector.z < 0){
    normal_vector_rel.vector.x = -normal_vector_rel.vector.x;
    normal_vector_rel.vector.y = -normal_vector_rel.vector.y;    
    normal_vector_rel.vector.z = -normal_vector_rel.vector.z;
  }

  // Transform the normal vector to fixed reference frame
  lst->waitForTransform(fixed_frame, input_cloud->header.frame_id, ros::Time::now(), ros::Duration(5.0));
  lst->transformVector(fixed_frame,normal_vector_rel, normal_vector_abs);
  */
  //try{lst->transformVector(fixed_frame,normal_vector_rel, normal_vector_abs);}
  //catch (tf::TransformException& ex) {}

  // ----------------------Compute centroid-----------------------------
  Eigen::Vector4f centroid_out;
  //Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*cloud_filtered_sor,centroid_out); 
  //centroid_in=centroid_out;

  geometry_msgs::PointStamped centroid_rel;
  geometry_msgs::PointStamped centroid_abs;
  centroid_rel.header.frame_id = input_cloud->header.frame_id;
  centroid_rel.header.stamp = ros::Time::now();

  centroid_rel.point.x = centroid_out[0];
  centroid_rel.point.y = centroid_out[1];
  centroid_rel.point.z = centroid_out[2];

  
  // Transform centroid to fixed reference frame
  lst->waitForTransform(fixed_frame, input_cloud->header.frame_id, ros::Time::now(), ros::Duration(4.0));
  lst->transformPoint(fixed_frame,centroid_rel, centroid_abs);
  //try{lst->transformPoint(fixed_frame,centroid_rel, centroid_abs);}
  //catch (tf::TransformException& ex) {}


  // ---------------Create detected object reference frame-------------

  geometry_msgs::PoseStamped object_pose;

  object_pose.pose.position.x = centroid_abs.point.x;
  object_pose.pose.position.y = centroid_abs.point.y;
  object_pose.pose.position.z = centroid_abs.point.z;
  

  ObjInfo cur_obj;
  cur_obj.x = xmin;
  cur_obj.y = ymin;
  cur_obj.width = xmax-xmin;
  cur_obj.height = ymax-ymin;
  cur_obj.label = object_name;
  cur_obj.last_time = ros::Time::now();
  cur_obj.average_depth=avg_depth;
  cur_obj.no_observation=false;
  cur_obj.center = centroid_abs;

  /*
  // Project normal vector in the floor plane
  normal_vector_abs.vector.z = 0.0;

  // Vertical vector
  geometry_msgs::Vector3 vertical_vector;
  vertical_vector.x = 0.0; vertical_vector.y = 0.0; vertical_vector.z = 1.0;
  
  // normalize vectors
  float module_normal = sqrt(normal_vector_abs.vector.x*normal_vector_abs.vector.x + normal_vector_abs.vector.y*normal_vector_abs.vector.y);
  normal_vector_abs.vector.x = normal_vector_abs.vector.x/module_normal; normal_vector_abs.vector.y = normal_vector_abs.vector.y/module_normal; 

  // do cross product
  geometry_msgs::Vector3 cross_p;
  
  cross_p.x = normal_vector_abs.vector.y;
  cross_p.y = -normal_vector_abs.vector.x;
  cross_p.z = 0.0;


  // calculate determinant
  //float determinant = -cross_p.x*normal_vector_abs.vector.y+normal_vector_abs.vector.x*cross_p.y;

  // make sure the orientation of the base is correct
  //cross_p.x = cross_p.x*determinant; cross_p.y = cross_p.y*determinant;

  // compute rotation matrix
  tf::Matrix3x3 rotation_matrix(0.0,cross_p.x,normal_vector_abs.vector.x,0.0,cross_p.y,normal_vector_abs.vector.y,1.0,0.0,0.0);

  // compute quaternion
  tf::Quaternion quaternion;
  rotation_matrix.getRotation(quaternion);

  // set orientation of the object
  tf::Stamped<tf::Transform> object_orientation(tf::Transform(quaternion), ros::Time::now(), fixed_frame);
  tf::poseStampedTFToMsg(object_orientation,object_pose);

  object_pose.pose.position.x = centroid_abs.point.x;
  object_pose.pose.position.y = centroid_abs.point.y;
  object_pose.pose.position.z = centroid_abs.point.z;

  */
  // ---------------Store resultant cloud and centroid------------------
  if (object_name == "bottle"){
    *cloud_bottle += *cloud_filtered_outplane;
    centroid_bottle_list.points.push_back(centroid_abs.point);
    handle_list.position.push_back(centroid_abs);
    bottle_poses.poses.push_back(object_pose.pose);

  }
  else if (object_name == "cup"){
    *cloud_cup += *cloud_filtered_inplane;
    centroid_cup_list.points.push_back(centroid_abs.point);
    cup_poses.poses.push_back(object_pose.pose);
  }
  else{
    //*cloud_refrigeratordoor += *cloud_filtered_inplane;
    //centroid_refrigeratordoor_list.points.push_back(centroid_abs.point);
    //refrigeratordoor_poses.poses.push_back(object_pose.pose);
  }

    // If the label does not yet exist, add it
    // Else if the label exist, check the position
    auto it = labels_to_obj.find(object_name);
    if (it == labels_to_obj.end()) {

        labels_to_obj.insert({object_name, cur_obj});
    }
    else
    {
        //update new information
        it->second= cur_obj;
    }
  }






   //erase index if survival time passed
   std::vector<int> erase_index;
   ros::Time current_time = ros::Time::now();

   ROS_INFO("----current label & id to position---" );
   std::map<std::string, ObjInfo >::iterator mapiter
                                            =labels_to_obj.begin(); 
   for(mapiter; mapiter != labels_to_obj.end(); mapiter++)
   {
        //labellist.push_back(mapiter->first);
        auto obj_time=mapiter->second.last_time;
        auto obj_depth= mapiter->second.average_depth;

        //ROS_INFO("label: %s, id:%d, position x: %.3lf, y: %.3lf, z: %.3lf,d: %.3lf",
        //mapiter->first.c_str(), (mapiter->second)[j], (idx->second).x, (idx->second).y, (idx->second).z, (iddepth->second));

        float dur = (current_time-obj_time).toSec();
        ROS_INFO("current duration for object %s :: %.3lf",mapiter->first.c_str(), dur );

        if(dur>TEMP_OCC_TIME && Is_target(mapiter->first))
        {
            ROS_INFO("target class %s is not detected!!! should check the depthime",  mapiter->first.c_str() );
            //calucate depth from last detected bounding box
            //----------------------------------------------------------------------------------
            pcl::PointIndices::Ptr inliers_roi_new (new pcl::PointIndices ());
            // Get inliers
            std::vector<int> indices_new;
            for (int column(mapiter->second.x); column<= mapiter->second.x+mapiter->second.width; column++){
                for (int row(mapiter->second.y); row<=mapiter->second.y+mapiter->second.height; row++){
                    // Pixel coordinates to pointcloud index
                    indices_new.push_back(row*mapiter->second.width+column);
                }
            }

            inliers_roi_new->indices = indices_new;

            // Create the filtering ROI object
            pcl::ExtractIndices<pcl::PointXYZRGB> extract_roi_new;
            // Extract the inliers  of the ROI
            extract_roi_new.setInputCloud (cloud);
            extract_roi_new.setIndices (inliers_roi_new);
            extract_roi_new.setNegative (false);
            extract_roi_new.filter (*cloud_filtered_roi);

            // Create the filtering object
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_noise;
            // Remove noise
            sor_noise.setInputCloud (cloud_filtered_roi);
            //sor_noise.setInputCloud (cloud_filtered_voxelgrid);
            sor_noise.setMeanK (50);
            sor_noise.setStddevMulThresh (1.0);
            sor_noise.filter (*cloud_filtered_sor);

            //remove NaN points from the cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr nanfiltered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<int> rindices;
            pcl::removeNaNFromPointCloud(*cloud_filtered_sor,*nanfiltered_cloud, rindices);

            float avg_depth =0;
            float nearest=0.0;
            //depth_maxcount=0.0;
            int depth_count =nanfiltered_cloud->points.size();
            for(size_t i=0;i<nanfiltered_cloud->points.size();i++)
            {
                //std::cout<<"inside clouds"<<nanfiltered_cloud->points[i].z<<std::endl;
                float cur_depths = nanfiltered_cloud->points[i].z;
                avg_depth+=nanfiltered_cloud->points[i].z;
                nearest =roundf(cur_depths*1000) /1000.0;
            }

            avg_depth =avg_depth/depth_count;
            ROS_INFO("object: %s, before depth : %.3lf,  new_depth : %.3lf",mapiter->first.c_str(),  mapiter->second.average_depth, avg_depth);
            //--------------------------------------------------------------------------------------

            
            //calculate overlapratio among detected boundboxes


        }








        if(dur>SURVIVAL_TIME)
        {
            labels_to_obj.erase(mapiter->first);
            ROS_INFO("class %s is erased!!! --out of time",  mapiter->first.c_str() );
        }




    }


  // Create a container for the result data.
  sensor_msgs::PointCloud2 output_bottle;
  sensor_msgs::PointCloud2 output_cup;

  // Convert pcl::PointCloud to sensor_msgs::PointCloud2
  pcl::toROSMsg(*cloud_bottle,output_bottle);
  pcl::toROSMsg(*cloud_cup,output_cup);

  // Set output frame as the input frame
  output_bottle.header.frame_id           = input_cloud->header.frame_id;
  output_cup.header.frame_id      = input_cloud->header.frame_id;

  // Publish the data.
  pub_bottle.publish (output_bottle);
  pub_cup.publish (output_cup);

  // Publish markers
  pub_bottle_centroid.publish (centroid_bottle_list);
  pub_cup_centroid.publish (centroid_cup_list);

  // Publish handle position
  pub_handle_centroid_list.publish (handle_list);

  // Publish poses
  pub_bottle_poses.publish(bottle_poses);
  pub_cup_poses.publish(cup_poses);

  //ROS_INFO("Pointcloud processed");
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_processing");
  ros::NodeHandle nh;

  target_string.push_back("bottle");
  target_string.push_back("cup");

  std::deque<float> depthvector;
  depth_buffer.insert({"bottle", depthvector});
  depth_buffer.insert({"cup", depthvector});


  // Initialize subscribers to darknet detection and pointcloud
  //message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud(nh, "hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud(nh, "camera/depth_registered/points", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_box(nh, "darknet_ros/bounding_boxes", 1);

  // Initialize transform listener
  tf::TransformListener listener(ros::Duration(10));
  lst = &listener;

  // Synchronize darknet detection with pointcloud
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_cloud, sub_box);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  // Create a ROS publisher for the output point cloud
  pub_bottle = nh.advertise<sensor_msgs::PointCloud2> ("hsrb/head_rgbd_sensor/depth_registered/rectified_points/bottle", 1);
  pub_cup = nh.advertise<sensor_msgs::PointCloud2> ("hsrb/head_rgbd_sensor/depth_registered/rectified_points/cup", 1);

  // Create a ROS publisher for the output point cloud centroid markers
  pub_bottle_centroid = nh.advertise<visualization_msgs::Marker> ("hsrb/head_rgbd_sensor/depth_registered/rectified_points/bottle_centroid", 1);
  pub_cup_centroid = nh.advertise<visualization_msgs::Marker> ("hsrb/head_rgbd_sensor/depth_registered/rectified_points/cup_centroid", 1);

  // Create a ROS publisher for the detected poses
  pub_bottle_poses = nh.advertise<geometry_msgs::PoseArray> ("/bottle_poses", 1);
  pub_cup_poses = nh.advertise<geometry_msgs::PoseArray> ("/cup_poses", 1);

  // Create a ROS publisher for handle position
  pub_handle_centroid_list = nh.advertise<pointcloud_processing_msgs::handle_position> ("detected_bottle", 1);

  // Spin
  ros::spin ();
}
