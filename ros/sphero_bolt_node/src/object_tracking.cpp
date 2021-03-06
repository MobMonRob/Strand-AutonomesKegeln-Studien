#include <ros/ros.h>
#include <ros/console.h>
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


  public:
  PositionDetection() {
    sub = nh.subscribe("cloud", 1, &PositionDetection::callback, this);
    publisherPosition = nh.advertise<geometry_msgs::Point32>("/ball_position", 1);
    noPositionPublisher = nh.advertise<std_msgs::Bool>("/no_ball_detected", 1);

    targetPositionPublisher  = nh.advertise<geometry_msgs::Point32>("/target", 1);
  }


  void 
  callback (const sensor_msgs::PointCloud2ConstPtr& input)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    auto clusters = getClusters(temp_cloud);

    ROS_INFO_STREAM("clusters found: " << clusters.size());

    if (clusters.size() == 0)
      return;

    try {
    auto ballCluster = getBallCluster(clusters);
    auto ballLocation = getBallClusterLocation(ballCluster); 
    lastBallDetection = ros::Time::now().toSec();
     
    ROS_INFO_STREAM("found ballcluster with size: " <<  ballCluster.points.size() << " x: " << ballLocation.x  << 
    " y: " << ballLocation.y);

    auto output = geometry_msgs::Point32();
    output.x = ballLocation.x;
    output.y = ballLocation.y;
    publisherPosition.publish(output);
    } catch (const std::exception& err) {
	  ROS_INFO("No ball detected");
      auto output = std_msgs::Bool();
      output.data = true;
      noPositionPublisher.publish(output);

      //haven't found the ball in more than one second
      if (ros::Time::now().toSec() - lastBallDetection > 1.0) {

        auto targetCluster = getClusterWithMostPoints(clusters);
        auto targetLocation = getTargetLocation(targetCluster);

        ROS_INFO_STREAM("Found target cluster with size: " << targetCluster.points.size() << " x: " << targetLocation.x << 
        " y: " << targetLocation.y);

        auto targetOutput = geometry_msgs::Point32();
        targetOutput.x = targetLocation.x;
        targetOutput.y = targetLocation.y;
        targetOutput.z = 0;
        targetPositionPublisher.publish(targetOutput);
      }
    }
  }


  private:
  std::vector<Cluster> getClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr points) {

    std::vector<Cluster> clusters;
    auto currentCluster = Cluster();

    for(auto& point: points->points) {
      if (point.x == 0 && point.y == 0 ) {
        continue;
      }
      if (!pointIsInField(point))
        continue;

      float distanceToLastPoint = 0;
      if (currentCluster.points.size() > 0)
        distanceToLastPoint =  distanceBetweenTwoPoints(point, currentCluster.points.back());

      if (distanceToLastPoint > EPSILON_POINT_DISTANCE) {
        //filter out clusters which consist of less than four points
        if (currentCluster.points.size() >= minimumClusterSize)
          clusters.push_back(currentCluster);
        currentCluster = Cluster();
      }
      currentCluster.points.push_back(pcl::PointXYZ(point));
    }
    if (currentCluster.points.size() >= minimumClusterSize)
      clusters.push_back(currentCluster);

    return clusters;
  }

  bool pointIsInField(const pcl::PointXYZ& point) {
    const float yOffset = FIELDSIZE_Y / 2.0f;
    if (yOffset < abs(point.y))
      return false;
    if (FIELDSIZE_X < abs(point.x))
      return false;

    return true;
  }


  const Cluster& getBallCluster(const std::vector<Cluster>& clusters) {
    for(const auto& cluster: clusters) {
      const auto distance = getClusterDistanceToOrigin(cluster);
      const auto expectedPoints = expectedBallPoints(distance);
      ROS_INFO_STREAM("distance: " << distance << " expected points: " << expectedPoints << " actual points: " << cluster.points.size());

      if (expectedPoints - clusterPointTolerance  <= cluster.points.size() && cluster.points.size() <= expectedPoints + clusterPointTolerance ) {
        return cluster;
      }
    }
    throw std::invalid_argument("Ball not found");
  }
  
  float distanceBetweenTwoPoints(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2) {
    return sqrtf(powf(point1.x - point2.x, 2.0) + powf(point1.y - point2.y, 2.0));
  }


  float getClusterDistanceToOrigin(const Cluster& cluster) {
    auto clusterLocation = getClusterLocation(cluster);
    return distanceBetweenTwoPoints(origin, clusterLocation);
  }


  int expectedBallPoints(float sensorDistance) {
    return (int) (spheroDiameterAtOpticalAxis / distanceBetweenMeassurementPoints(sensorDistance));
  }


  pcl::PointXYZ getBallClusterLocation(const Cluster& cluster) {
    pcl::PointXYZ result = cluster.points[0];
    for (int i = 1; i < cluster.points.size(); i++) {
      if (cluster.points[i].x < result.x)
        result = cluster.points[i];
    }
    return result;
  }

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
    
    return pcl::PointXYZ(x, y, z);
  }


  float distanceBetweenMeassurementPoints(float sensorDistance) {
    return sensorDistance * tan(sensorAngleResolution*radiantToDegreeFactor);
  }


  const Cluster& getClusterWithMostPoints(const std::vector<Cluster>& clusters) {
    int maxPoints = 0;
    int targetIndex = 0;
    for (int i = 0; i < clusters.size(); i++) {
      if (clusters[i].points.size() > maxPoints) {
        targetIndex = i;
        maxPoints = clusters[i].points.size();
      }
    }
    return clusters[targetIndex];
  }

  pcl::PointXYZ getTargetLocation(const Cluster& targetCluster) {
    auto clusterLocation = getClusterLocation(targetCluster);
    auto result = targetCluster.points[0];
    //point is right to
    if (pointIsRightOfSensor(clusterLocation)) {
      for (int i = 0; i < targetCluster.points.size(); i++) {
        if (targetCluster.points[i].y > result.y)
          result = targetCluster.points[i];
      }
    } else {
      for (int i = 0; i < targetCluster.points.size(); i++) {
        if (targetCluster.points[i].y < result.y)
          result = targetCluster.points[i];
      }
    }
    return result;
  }
  
  bool pointIsRightOfSensor(const pcl::PointXYZ point) {
    return point.y < 0;
  }

  private:
  //adjustable variables
    const int clusterPointTolerance = 10;
    const int minimumClusterSize = 5;
    const float FIELDSIZE_X = 1.3f;
    const float FIELDSIZE_Y = 3.0f;
    const float EPSILON_POINT_DISTANCE = 0.035f;

    //non adjustable variables
    const pcl::PointXYZ origin = pcl::PointXYZ(0.0f, 0.0f, 0.0f);
    const float spheroRadius = 0.0365f; //36.5mm
    const float sensorAngleResolution = 0.16f;
    const float heightOpticalAxis = 0.063f; //63mm
    const float spheroDiameterAtOpticalAxis = 2*sqrtf(powf(spheroRadius, 2.0)-powf(spheroRadius - heightOpticalAxis, 2.0)); //2*sqrt(r^2-(r-h)^2)
    const float radiantToDegreeFactor = M_PI/180.0f;
    bool ballDetected = false;
    double lastBallDetection;

    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher publisherPosition;
    ros::Publisher noPositionPublisher;
    ros::Publisher targetPositionPublisher;
};


int main (int argc, char** argv)
{
  ros::init (argc, argv, "sensor_processing");
  PositionDetection positionDetection;

  ros::spin ();
}
