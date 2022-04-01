#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

        
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //convert input to pcl::PointCloud<pcl::PointXYZ>
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  for (int i = 0; i < temp_cloud->points.size(); i++) {

    std::cout << "x:\t" << temp_cloud->points[i].x << "\ty:\t" << temp_cloud->points[i].y << "\tz:\t" << temp_cloud->points[i].z << "\n";
  }
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "sensor_processing");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud", 1, cloud_cb);

  // Spin
  ros::spin ();
}
