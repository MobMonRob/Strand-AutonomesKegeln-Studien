
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


class TargetAngleControl {
    public:
    TargetAngleControl() {
        //fix position for development
        target.x = 0.681f;
        target.y = -1.07f;

        subscriberNoBallDetected = nh.subscribe("no_ball_detected", 1, &TargetAngleControl::callbackNoBallDetected, this);
        subscriberBallPosition = nh.subscribe("ball_position", 1, &TargetAngleControl::callbackBallPosition, this);
        publisherHeading = nh.advertise<std_msgs::Int16>("sphero_control/heading", 1);
        publisherSpeed = nh.advertise<std_msgs::Int16>("sphero_control/speed", 1);
    }

    void updateSpeed(int16_t newSpeed) {
        newSpeed = std::min(newSpeed, (int16_t)255);
        if (newSpeed == speed ) 
            return;

        std::cout << "publishing speed: " << newSpeed << std::endl;
        std_msgs::Int16 output;
        output.data = newSpeed;
        speed = newSpeed;
        publisherSpeed.publish(output);
    }

	void updateHeading(int16_t newHeading) {
		newHeading = std::min(newHeading, (int16_t)360);
		if (newHeading == heading)
			return;

        std::cout << "publishing heading: " << newHeading << std::endl;
        std_msgs::Int16 output;
        output.data = newHeading;
        heading = newHeading;
        publisherHeading.publish(output);
	}

    void callbackNoBallDetected(std_msgs::Bool input) {
        updateSpeed(0);
    }

    bool ballIsAboveTarget() {
        return ball_position.x >= target.x;
    }

    float getIdealAngle() {
        const float radiantToDegreeFactor = 180/M_PI; //deg/Rad = 360/2pi -> deg = pi*Rad/180
        float deltaX = fabs(ball_position.x - target.x);
        float deltaY = fabs(ball_position.y - target.y);
        float angle = radiantToDegreeFactor*atanf(deltaY/deltaX);

        std::cout << "deltax: " << deltaX << "deltay: " << deltaY << "angle: " << angle << std::endl;
        //0 degree means the sphero goes straight
        //for a detailed epxlanation look into the documentation
        if (ballIsAboveTarget()) {
            return 90.0f - angle;
        } else {
            return 360.0f - (90.0f - angle);
        }
    }


    void callbackBallPosition(geometry_msgs::Point32 input) {
        ball_position = input;
        int16_t idealAngle = (int16_t) getIdealAngle();

        std::cout << "x:\t" << input.x << "y:\t" << input.y << "z:\t" << input.z << std::endl;
        std::cout << "target angle: " << idealAngle << std::endl;
        std::cout << "------------------------------------" << std::endl;
        std_msgs::Int16 output;


        if (idealAngle == heading) {
            //updateSpeed(speed + 15);
            updateSpeed(speed + 5);
        } else {
            updateSpeed(15);
        }
		updateHeading(idealAngle);

    }

    private:
    int16_t speed = 0;
    int16_t heading = 0;
    geometry_msgs::Point32 ball_position;
    geometry_msgs::Point32 target;

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
