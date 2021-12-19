#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sstream>

#define MAX_POINTS 5
#define OBSTACLE_CONTROL_BUTTON 5
#define LEGGED_ACTIVATE_BUTTON 7
#define WHEELED_MODE "wheeled"
#define LEGGED_MODE "legged"


class ModeManager{
  private:
    ros::Publisher *pub_pc;
    ros::Publisher *pub_mode;
    std::string actual_mode = "wheeled";
    bool points_in_cloud;
    bool joy_obstacle_control_disable = true;
    bool joy_legged_active = false;
    int last_obstacle_control_button = 0;
    int last_legged_activate_button = 0;

  public:
    void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& input){

      if (joy_obstacle_control_disable == false){
        // Create XYZ objects
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);

        // Convert the PointCloud2
        pcl::PCLPointCloud2 raw_cloud;
        pcl_conversions::toPCL(*input, raw_cloud);
        pcl::fromPCLPointCloud2 (raw_cloud, *cloud);
        ROS_DEBUG("The original PointCloud has %lu points", cloud->points.size ());


        // Create the VoxelGrid object to filter PointCloud
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    
        voxel_grid.setInputCloud (cloud);
        voxel_grid.setLeafSize (0.01f, 0.01f, 0.01f); // Using leaf size of 1 cm
        voxel_grid.filter (*cloud_filtered);
        ROS_DEBUG("The filtered PointCloud has %lu points", cloud_filtered->points.size ());
      
        // Create the SACSegmentation object for the planar model
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
        segmentation.setOptimizeCoefficients (true);
        segmentation.setModelType (pcl::SACMODEL_PLANE);
        segmentation.setMethodType (pcl::SAC_RANSAC);
        segmentation.setEpsAngle(  30.0f * (3.1415/180.0f) );
        segmentation.setMaxIterations (10);
        segmentation.setDistanceThreshold (0.01);

        int nr_points = (int) cloud_filtered->points.size (); // Initial size of the cloud

        // While 30% of the original cloud is still there
        while (cloud_filtered->points.size () > 0.3 * nr_points)
        {
          // Segment the largest planar component from the remaining cloud
          segmentation.setInputCloud (cloud_filtered);
          segmentation.segment (*inliers, *coefficients);
          if (inliers->indices.size () == 0)
          {
            ROS_WARN("Was not possible to estimate a planar model for the pointCloud.");
            break;
          }

          // Extract the planar inliers from the input cloud
          pcl::ExtractIndices<pcl::PointXYZ> extract;
          extract.setInputCloud (cloud_filtered);
          extract.setIndices (inliers);
          extract.setNegative (false);

          // Get the points associated with the planar surface
          extract.filter (*cloud_plane);
          ROS_DEBUG("The PointCloud of the planar component has %lu points", cloud_plane->points.size ());

          // Remove the planar inliers, extract the rest
          extract.setNegative (true);
          extract.filter (*cloud_f);
          *cloud_filtered = *cloud_f;
        }

        // Create the PassThrough filtering object
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (0.0, 1);
        pass.filter (*cloud_p);
        pass.setInputCloud (cloud_p);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-0.3, 0.3);
        pass.filter (*cloud_p);
        pass.setInputCloud (cloud_p);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-0.6, 0.05);
        pass.filter (*cloud_p);
        *cloud_filtered = *cloud_p;

        pcl::PCLPointCloud2 outcloud;
        pcl::toPCLPointCloud2 (*cloud_filtered, outcloud);
        ros::Time time_st = ros::Time::now ();
        outcloud.header.stamp = time_st.toNSec()/1e3;
        outcloud.header.frame_id = "/aww/velodyne";
        this->pub_pc->publish(outcloud);

        // set the varibale to control robot model
        points_in_cloud = (outcloud.data.size() > MAX_POINTS) ? true : false;
      }
      else {
        points_in_cloud = false;
      }
    }

    void joyCallback (const sensor_msgs::Joy::ConstPtr& msg) {
      if ((msg->buttons[OBSTACLE_CONTROL_BUTTON] > 0) && (last_obstacle_control_button == 0)) {
        joy_obstacle_control_disable = !joy_obstacle_control_disable;
        if (joy_obstacle_control_disable != true)
          ROS_INFO(" ---- Activating obstacle control mode! ----");
        else 
          ROS_INFO(" ---- Activating direct control mode! ----");
      }
      if ((msg->buttons[LEGGED_ACTIVATE_BUTTON] > 0) && (last_legged_activate_button == 0)){
        joy_legged_active = !joy_legged_active;
      }
      last_obstacle_control_button = msg->buttons[OBSTACLE_CONTROL_BUTTON];
      last_legged_activate_button = msg->buttons[LEGGED_ACTIVATE_BUTTON];
    }

    void setPublishers(ros::Publisher *pub_pc, ros::Publisher *pub_mode){
        this->pub_pc = pub_pc;
        this->pub_mode = pub_mode;
    }

    void publishMode(){
      std_msgs::String msg;
      if (joy_obstacle_control_disable != true) {
        msg.data = (points_in_cloud == true) ? WHEELED_MODE : LEGGED_MODE;
      }
      else {
        msg.data = (joy_legged_active == false) ? WHEELED_MODE : LEGGED_MODE;
      }
      //ROS_INFO("yyyyyyyyyyyyyyyy %s %s", actual_mode, msg.data.c_str());
      if (actual_mode != msg.data) {
        actual_mode = msg.data;
        ROS_INFO("Setting robot mode to: %s", msg.data.c_str());
        this->pub_mode->publish(msg);
      }
    }
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "mode_manager");
  ros::NodeHandle nh;
  ros::Rate loop_rate(30);

  // mode manager instance
  ModeManager mm;

  // Create a ROS publisher/subscriber for the point cloud, joy and mode
  //ros::Subscriber sub_points = nh.subscribe ("/aww/velodyne/points", 1, &ModeManager::cloudCallback, &mm);  
  ros::Subscriber sub_joy = nh.subscribe ("/joy", 1, &ModeManager::joyCallback, &mm);
  ros::Publisher pub_pc = nh.advertise<pcl::PCLPointCloud2> ("/aww/velodyne/points/filtered", 1);
  ros::Publisher pub_mode = nh.advertise<std_msgs::String> ("/aww/mode", 1);


  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Set Publisher
  mm.setPublishers(&pub_pc, &pub_mode);

  ROS_INFO("Starting mode manager node!");

  while (ros::ok()){
    mm.publishMode();
    ros::spinOnce();
    loop_rate.sleep();
  }

   return 0;
}