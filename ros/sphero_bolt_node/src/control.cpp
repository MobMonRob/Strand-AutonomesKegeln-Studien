
#include <ros/ros.h>
// PCL specific includes
#include <std_msgs/Int16.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>

        
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cmath>



class TargetAngleControl {
    public:
    TargetAngleControl() {
        //fix position for development
        target.x = 0.628f;
        target.y = -0.782f;

        sub = nh.subscribe("ball_position", 1, &TargetAngleControl::callback, this);
        pub = nh.advertise<std_msgs::Int16>("sphero_control/heading", 1);
    }

    bool ballIsAboveTarget() {
        return ball_position.x >= target.x;
    }

    float idealAngle() {
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


    void callback(geometry_msgs::Point32 input) {
        ball_position = input;

        std::cout << "x:\t" << input.x << "y:\t" << input.y << "z:\t" << input.z << std::endl;
        std::cout << "target angle: " << idealAngle() << std::endl;
        std::cout << "------------------------------------" << std::endl;
        std_msgs::Int16 output;
        output.data = (int16_t) idealAngle();
        pub.publish(output);
    }

    private:
    geometry_msgs::Point32 ball_position;
    geometry_msgs::Point32 target;

    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
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
