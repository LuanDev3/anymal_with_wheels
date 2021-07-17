#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
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
      pcl::PCDWriter writer;
      segmentation.setOptimizeCoefficients (true);
      segmentation.setModelType (pcl::SACMODEL_PLANE);
      segmentation.setMethodType (pcl::SAC_RANSAC);
      segmentation.setMaxIterations (100);
      segmentation.setDistanceThreshold (0.02);

      int nr_points = (int) cloud_filtered->points.size (); // Initial size of the cloud
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

      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud_filtered);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance (5); // 2cm
      ec.setMinClusterSize (1000);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_filtered);
      ec.extract (cluster_indices);

      int j = 0;
      //cloud_filtered_blob = cloud_plane;
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
          cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
          //uint8_t r = 0, g = 255, b = 0;
          //uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
          //cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rgb);    
        
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        //ss << "cloud_cluster_" << j << ".pcd";
        //pcl::compute3DCentroid(*cloud_cluster, const pcl::PointIndices& *indices, Eigen::Matrix<float,4,1>& centroid);  
        
        pcl::PCLPointCloud2 outcloud;
        pcl::toPCLPointCloud2 (*cloud_cluster, outcloud);
        ros::Time time_st = ros::Time::now ();
        outcloud.header.stamp = time_st.toNSec()/1e3;
        outcloud.header.frame_id = "/aww/velodyne";
        this->pub_pc->publish(outcloud);
        j++;
      }
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