#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
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


class ModeManager{
  private:
   ros::Publisher *pub_pc;
   ros::Publisher *pub_mode;
   char* atual_mode;

  public:
    void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& input){

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
    }

    void setPublishers(ros::Publisher *pub_pc, ros::Publisher *pub_mode){
        this->pub_pc = pub_pc;
        this->pub_mode = pub_mode;
    }

    void publishMode(const char* mode){
      if (mode != atual_mode){
        std_msgs::String msg;
        msg.data = mode;
        ROS_INFO("mode -> %s", msg.data.c_str());
        atual_mode = (char*)mode;
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

  // Create a ROS publisher/subscriber for the point cloud and mode
  ros::Subscriber sub = nh.subscribe ("/aww/velodyne/points", 1, &ModeManager::cloudCallback, &mm);
  ros::Publisher pub_pc = nh.advertise<pcl::PCLPointCloud2> ("test", 1);
  ros::Publisher pub_mode = nh.advertise<std_msgs::String> ("/aww/mode", 1);


  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Set publisher
  mm.setPublishers(&pub_pc, &pub_mode);

  while (ros::ok()){
    mm.publishMode("wheeled");
    ros::spinOnce();
    loop_rate.sleep();
  }

   return 0;
}