#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>
        
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cmath>
#include <utility>
#include <stdexcept>


struct Cluster {
  std::vector<pcl::PointXYZ> points;  
};

class PositionDetection {
  private:
  const pcl::PointXYZ origin = pcl::PointXYZ(0.0f, 0.0f, 0.0f);
  const float spheroRadius = 0.0365f; //36.5mm
  const float sensorAngleResolution = 0.16f;
  const float heightOpticalAxis = 0.063; //63mm
  const float spheroDiameterAtOpticalAxis = 2*sqrtf(powf(spheroRadius, 2.0)-powf(spheroRadius - heightOpticalAxis, 2.0)); //2*sqrt(r^2-(r-h)^2)
  const int clusterPointTolerance = 4;

  const float FIELDSIZE_X = 1.35f;
  const float FIELDSIZE_Y = 3.0f;
  const float EPSILON_POINT_DISTANCE = 0.035f;

  bool ballDetected = false;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher publisherPosition;
  ros::Publisher noPositionPublisher;


  public:
  PositionDetection() {
    sub = nh.subscribe("cloud", 1, &PositionDetection::callback, this);
    publisherPosition = nh.advertise<geometry_msgs::Point32>("/ball_position", 1);
    noPositionPublisher = nh.advertise<std_msgs::Bool>("/no_ball_detected", 1);
  }

  private:
  const pcl::PointXYZ getClusterLocation(const Cluster& cluster) {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    for (auto& point: cluster.points) {
      x += point.x;
      y += point.y;
    }
    x = x / cluster.points.size();
    y = y / cluster.points.size();
    z = z / cluster.points.size();
    
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

  float getClusterDistanceToOrigin(const Cluster& cluster) {
    auto clusterLocation = getClusterLocation(cluster);
    return sqrtf(powf(clusterLocation.x, 2.0) + powf(clusterLocation.y, 2.0));
  }

  float distanceBetweenMeassurementPoints(float sensorDistance) {
    //a1 = distance * tan alpha1 -> a = 2*a1 = 2 * distance * tan alpha1
    const float alpha1 = sensorAngleResolution / 2.0f;
    //return 2*sensorDistance*tan(alpha1 *(M_PI/180.0f));
    return sensorDistance * tan(sensorAngleResolution*(M_PI/180.0f));
  }

  int expectedBallPoints(float sensorDistance) {
    return (int) (spheroDiameterAtOpticalAxis / distanceBetweenMeassurementPoints(sensorDistance));
  }

  const Cluster& getBallCluster(const std::vector<Cluster>& clusters) {
    for(const auto& cluster: clusters) {
      const auto distance = getClusterDistanceToOrigin(cluster);
      const auto expectedPoints = expectedBallPoints(distance);
      std::cout << "distance: " << distance << "\texpected points: " << expectedPoints << "actual points: " << cluster.points.size() << std::endl;

      if (expectedPoints - clusterPointTolerance  <= cluster.points.size() && cluster.points.size() <= expectedPoints + clusterPointTolerance ) {
        return cluster;
      }
    }
    throw std::invalid_argument("Ball not found");
  }

  void 
  callback (const sensor_msgs::PointCloud2ConstPtr& input)
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
        //filter out clusters which consist of less than four points
        if (currentCluster.points.size() >= 4)
          clusters.push_back(currentCluster);
        currentCluster = Cluster();
      }
      currentCluster.points.push_back(pcl::PointXYZ(point));
    }
    if (currentCluster.points.size() >= 4)
      clusters.push_back(currentCluster);

    std::cout << "clusters: " << clusters.size() << " " << std::endl;

    //lol

    try {
    auto ballCluster = getBallCluster(clusters);
    auto ballLocation = getClusterLocation(ballCluster); 


    std::cout << "found ballcluster with size: " << ballCluster.points.size() << "\nx:\t" << ballLocation.x  << 
    "\ty:\t" << ballLocation.y << "\tz:\t" << ballLocation.z << std::endl;

    auto output = geometry_msgs::Point32();
    output.x = ballLocation.x;
    output.y = ballLocation.y;
    output.z = ballLocation.z;
    publisherPosition.publish(output);
    } catch (const std::exception& err) {
      auto output = std_msgs::Bool();
      output.data = true;
      noPositionPublisher.publish(output);
    }
  }
};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "sensor_processing");
/*  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud", 1, cloud_cb);
  ros::Publisher pub = nh.advertise<geometry_msgs::Point32>("/ball_position");*/
  // Spin
  PositionDetection positionDetection;

  ros::spin ();
}
