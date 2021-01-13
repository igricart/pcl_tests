#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub_voxel_grid_filter;
ros::Publisher pub_2;
ros::Publisher pub_3;
ros::Publisher pub;

void cloud_cb_extract_indices (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::PCLPointCloud2::Ptr cloud_filtered_ptr(new pcl::PCLPointCloud2);

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud);
  
  // Create Voxel Grid
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*cloud_filtered_ptr);

  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 output_voxel_grid_filter;
  sensor_msgs::PointCloud2 output_2;
  sensor_msgs::PointCloud2 output_3;
  pcl_conversions::fromPCL(*cloud_filtered_ptr, output_voxel_grid_filter);
  pub_voxel_grid_filter.publish(output_voxel_grid_filter);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Make cloud const ptr
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_const_ptr (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(*cloud_filtered_ptr, *cloud_const_ptr);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  
  int i = 0, nr_points = (int) cloud_const_ptr->size ();
  while (cloud_const_ptr->size () > 0.3 * nr_points)
  {
    seg.setInputCloud(cloud_const_ptr);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    // Create the filtering object

    extract.setInputCloud(cloud_const_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);

    // Obtain images without floor
    pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr pcl2_extracted_cloud_ptr(new pcl::PCLPointCloud2);
    extract.filter(*extracted_cloud_ptr);
    pcl::toPCLPointCloud2(*extracted_cloud_ptr, *pcl2_extracted_cloud_ptr);
    if (i == 0)
    {
      pcl_conversions::fromPCL(*pcl2_extracted_cloud_ptr, output_2);
      pub_2.publish(output_2);
    } else if (i == 1)
    {
      pcl_conversions::fromPCL(*pcl2_extracted_cloud_ptr, output_3);
      pub_3.publish(output_3);
    } else
    {
      std::cout << "Not handled case of multiples filters" << std::endl;
    }
    
    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZ>::Ptr rest_extracted_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative (false);
    extract.filter (*rest_extracted_cloud_ptr);
    cloud_const_ptr.swap (rest_extracted_cloud_ptr);

    i++;
    
  }
}

void cloud_cb_model_plane (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (inliers, coefficients);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub.publish (ros_coefficients);
}

void cloud_cb_model_cylinder (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (inliers, coefficients);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub.publish (ros_coefficients);
}

void cloud_cb_do_nothing (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

void cloud_cb_filter_voxel_grid (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb_extract_indices);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_voxel_grid_filter = nh.advertise<sensor_msgs::PointCloud2> ("output_voxel_grid_filter", 1);
  pub_2 = nh.advertise<sensor_msgs::PointCloud2> ("output_2", 1);
  pub_3 = nh.advertise<sensor_msgs::PointCloud2> ("output_3", 1);

  // Create a ROS publisher for the output model coefficients
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  // Spin
  ros::spin ();
}