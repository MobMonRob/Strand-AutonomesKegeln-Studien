
#include <ros/ros.h>
// PCL specific includes
#include <std_msgs/Int16.h>
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
#include <algorithm>

#include "FuturePositionPrediction.h"
/*struct BufferedPosition {
    pcl::PointXYZ position;
    double timeStamp;
    bool ballFound = false;
};*/

class TargetAngleControl {
    public:
    TargetAngleControl() {
        //fix position for development
        target.x = 0.893f;
        target.y = -1.35f;

        subscriberNoBallDetected = nh.subscribe("no_ball_detected", 1, &TargetAngleControl::callbackNoBallDetected, this);
        subscriberBallPosition = nh.subscribe("ball_position", 1, &TargetAngleControl::callbackBallPosition, this);
        publisherHeading = nh.advertise<std_msgs::Int16>("sphero_control/heading", 1);
        publisherSpeed = nh.advertise<std_msgs::Int16>("sphero_control/speed", 1);
    }

    void updateSpeed(int16_t newSpeed) {
        newSpeed = std::max((int16_t)0, std::min(newSpeed, (int16_t)255));
        if (newSpeed == speed) 
            return;

        ROS_INFO_STREAM("publishing speed: " << newSpeed);
        std_msgs::Int16 output;
        output.data = newSpeed;
        speed = newSpeed;
        publisherSpeed.publish(output);
    }

	void updateHeading(int16_t newHeading) {
		newHeading = std::min(newHeading, (int16_t)360);
			
        if (heading - 1 <= newHeading && newHeading <= heading + 1) 
			return;

        ROS_INFO_STREAM("publishing heading: " << newHeading);
        std_msgs::Int16 output;
        output.data = newHeading;
        heading = newHeading;
        publisherHeading.publish(output);
	}

    void callbackNoBallDetected(std_msgs::Bool input) {
		ROS_INFO("No ball detected");
        BufferedPosition positionToBeBuffered;
        positionToBeBuffered.ballFound = false;
        positionPredictor.add(positionToBeBuffered);
        updateSpeed(speed - 15);
    }

    bool positionIsAboveTarget(pcl::PointXYZ position, pcl::PointXYZ target) {
        return position.x >= target.x;
    }

    float getIdealAngle(pcl::PointXYZ position, pcl::PointXYZ target) {
        const float radiantToDegreeFactor = 180/M_PI; //deg/Rad = 360/2pi -> deg = pi*Rad/180
        float deltaX = fabs(position.x - target.x);
        float deltaY = fabs(position.y - target.y);
        float angle = radiantToDegreeFactor*atanf(deltaY/deltaX);

        ROS_INFO_STREAM("deltax: " << deltaX << "deltay: " << deltaY << "angle: " << angle);
        //0 degree means the sphero goes straight
        //for a detailed epxlanation look into the documentation
        if (positionIsAboveTarget(position, target)) {
            return 90.0f - angle;
        } else {
            return 360.0f - (90.0f - angle);
        }
    }

    /*pcl::PointXYZ getFuturePosition(pcl::PointXYZ currentPosition, double timeStamp) {
        if (!lastPosition.ballFound)
            return currentPosition;
        
        double time = timeStamp - lastPosition.timeStamp;
        float xVelocity = (currentPosition.x - lastPosition.position.x) / time;
        float yVelocity = (currentPosition.y - lastPosition.position.y) / time;
        pcl::PointXYZ futurePosition(currentPosition.x + xVelocity * time, currentPosition.y + yVelocity * time, currentPosition.z); 

        double absoluteVelocity = sqrtf(powf(xVelocity, 2.0) + powf(yVelocity, 2.0)); 
        ROS_INFO_STREAM("Ball belocity: " << absoluteVelocity);

        return futurePosition;
    }*/

    void callbackBallPosition(geometry_msgs::Point32 input) {
        pcl::PointXYZ ballPosition = pcl::PointXYZ(input.x, input.y, input.z);
        double timeStamp = ros::Time::now().toSec();

        auto future_position = positionPredictor.predictPosition(ballPosition, timeStamp);

        int16_t idealAngle = (int16_t) getIdealAngle(future_position, target);
        std_msgs::Int16 output;
		

        ROS_INFO_STREAM("Position x: " << input.x << " y: " << input.y << " z: " << input.z
        << "ideal heading: " << idealAngle);

        if (heading - 1 <= idealAngle && idealAngle <= heading + 1) {
            //updateSpeed(speed + 15);
            updateSpeed(speed + 2);
        } else {
            updateSpeed(15);
        }
		updateHeading(idealAngle);

        BufferedPosition positionToBeBuffered;
        positionToBeBuffered.ballFound = true;
        positionToBeBuffered.position = ballPosition;
        positionToBeBuffered.timeStamp = timeStamp;
        positionPredictor.add(positionToBeBuffered);
    }

    private:
    int16_t speed = 0;
    int16_t heading = 0;
    pcl::PointXYZ target;

    FuturePositionPrediction positionPredictor;

    ros::NodeHandle nh;
    ros::Subscriber subscriberBallPosition;
    ros::Subscriber subscriberNoBallDetected;
    ros::Publisher publisherSpeed;
    ros::Publisher publisherHeading;
};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "control");
/*  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud", 1, cloud_cb);
  ros::Publisher pub = nh.advertise<geometry_msgs::Point32>("/ball_position");*/
  // Spin
  TargetAngleControl targetAngleControl;

  ros::spin ();
}
