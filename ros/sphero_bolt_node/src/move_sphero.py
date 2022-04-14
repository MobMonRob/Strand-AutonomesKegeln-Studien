#!/usr/bin/env python

import queue
import rospy
import time

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color
from sphero_bolt_node.msg import Roll
from std_msgs.msg import  Int16, Empty
#import ros standard message int 

class SpheroControl:
    def __init__(self, bolt) -> None:
        self.bolt = bolt
        self.bolt.reset_aim()
        self.bolt.set_main_led(Color(r=0, g=255, b=0)) #Sets whole Matrix

        rospy.Subscriber('sphero_control/roll', Roll, self.callbackRoll, bolt, queue_size=1)
        rospy.Subscriber('sphero_control/heading', Int16, self.callbackHeading, queue_size=1)
        rospy.Subscriber('sphero_control/speed', Int16, self.callbackSpeed, queue_size=1)
        rospy.Subscriber('sphero_control/stopRoll', Empty, self.callbackStopRoll, queue_size=1)


    def callbackRoll(self, data):
        rospy.loginfo(f'rolling message: {data}')
        self.bolt.roll(min(data.heading, 360), min(data.speed, 255), data.duration)

    def callbackStopRoll(self, data):
        rospy.loginfo(f'stop')
        self.bolt.stop_roll()


    def callbackHeading(self, data):
        rospy.loginfo(f'heading message. {data}')
        newHeading = min(data.data, 360)
        self.bolt.set_heading(newHeading)

    def callbackSpeed(self, data):
        rospy.loginfo(f'speed message. {data}')
        newSpeed = min(data.data, 255)
        self.bolt.set_speed(newSpeed)


def main():
    rospy.loginfo("Connecting to Bolt...")

    toy = scanner.find_BOLT()
        
    if toy is not None:
        print("connected")
        with SpheroEduAPI(toy) as bolt:
            rospy.init_node('sphero', anonymous=True)
            control = SpheroControl(bolt)
            rospy.loginfo('listener node is up')
            rospy.spin()


if __name__ == '__main__':
    main()
