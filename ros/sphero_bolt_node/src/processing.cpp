#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  std::cout << "received something" << std::endl;


}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "sensor_processing");
  ros::NodeHandle nh;

  std::cout << "wer das lies ist verwirrt!" << std::endl;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("SENSORTOPICNAME", 1, cloud_cb);

  // Spin
  ros::spin ();
}
