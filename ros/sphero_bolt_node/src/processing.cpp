#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

        
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <iostream>
#include <vector>
#include <stdlib.h>

const float FIELDSIZE_X = 1.35f;
const float FIELDSIZE_Y = 3.0f;
const float EPSILON_POINT_DISTANCE = 0.035f;

struct Cluster {
  std::vector<pcl::PointXYZ> points;  
};

const pcl::PointXYZ getBallLocation(const Cluster& ballCluster) {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  for (auto& point: ballCluster.points) {
    x += point.x;
    y += point.y;
  }
  x = x / ballCluster.points.size();
  y = y / ballCluster.points.size();
  z = z / ballCluster.points.size();
  
  return pcl::PointXYZ(x, y, z);
}

bool pointIsInField(const pcl::PointXYZ& point) {
  const float yOffset = FIELDSIZE_Y / 2.0f;
  if (yOffset < abs(point.y))
    return false;
  if (FIELDSIZE_X < abs(point.x))
    return false;

  return true;
}


void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //convert input to pcl::PointCloud<pcl::PointXYZ>
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  std::vector<Cluster> clusters;
  auto currentCluster = Cluster();
  

  //build clusters
  for(auto& point: temp_cloud->points) {
    if (point.x == 0 && point.y == 0 && point.z == 0) {
      continue;
    }
    if (!pointIsInField(point))
      continue;

    float distanceToLastPoint = 0;
    if (currentCluster.points.size() > 0)
      distanceToLastPoint =  pcl::euclideanDistance(point, currentCluster.points.back());

    if (distanceToLastPoint > EPSILON_POINT_DISTANCE) {
      //filter out clusters which consist of only one point
      if (currentCluster.points.size() > 1)
        clusters.push_back(currentCluster);
      currentCluster = Cluster();
    }
    currentCluster.points.push_back(pcl::PointXYZ(point));
  }
  if (currentCluster.points.size() > 1)
    clusters.push_back(currentCluster);


  std::cout << "clusters: " << clusters.size() << " " << std::endl;
  for(const auto& cluster: clusters) {
    auto ballPosition = getBallLocation(cluster);
    std::cout << "ball position x:\t" << ballPosition.x << "\ty:\t" << ballPosition.y << "\tz:\t" << ballPosition.z << std::endl;
    std::cout << "points in cluster:\t" << cluster.points.size()
     << "first point:\t" << "x:\t" << cluster.points.front().x 
     << "\ty:\t" << cluster.points.front().y 
     << "\tz:\t" << cluster.points.front().z << std::endl;
    std::cout << "----------------------------------" << std::endl;
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
