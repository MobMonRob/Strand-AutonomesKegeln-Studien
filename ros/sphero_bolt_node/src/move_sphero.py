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


def callbackRoll(data, bolt):
    rospy.loginfo(f'rolling message: {data}')
    bolt.roll(data.heading, data.speed, data.duration)

def callbackStopRoll(data, bolt):
    rospy.loginfo(f'stop')
    bolt.stop_roll()


def callbackHeading(data, bolt):
    rospy.loginfo(f'heading message. {data}')
    newHeading = data.data
    if newHeading == heading:
        return
    else:
        heading = newHeading
        bolt.set_heading(newHeading)

def callbackSpeed(data, bolt):
    rospy.loginfo(f'speed message. {data}')
    newSpeed = data.data
    if newSpeed == speed:
        return
    else:
        speed = newSpeed
        bolt.set_speed(newSpeed)


def listener(bolt):

# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
    rospy.init_node('sphero', anonymous=True)
    rospy.loginfo('listener node is up')
    rospy.Subscriber('sphero_control/roll', Roll, callbackRoll, bolt, 1, 1)
    rospy.Subscriber('sphero_control/heading', Int16, callbackHeading, bolt, 1, 1)
    rospy.Subscriber('sphero_control/speed', Int16, callbackSpeed, bolt, 1, 1)
    rospy.Subscriber('sphero_control/stopRoll', Empty, callbackStopRoll, bolt, 1, 1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def initialize_bolt(bolt):
    bolt.set_main_led(Color(r=0, g=255, b=0)) #Sets whole Matrix
    bolt.reset_aim()

class SpheroControl:
    def __init__(self, bolt) -> None:
        self.bolt = bolt
        self.bolt.reset_aim()
        self.bolt.set_main_led(Color(r=0, g=255, b=0)) #Sets whole Matrix
        self.heading = 0
        self.speed = 0

        rospy.Subscriber('sphero_control/roll', Roll, self.callbackRoll, bolt, queue_size=1, buff_size=1)
        rospy.Subscriber('sphero_control/heading', Int16, self.callbackHeading, queue_size=1, buff_size=1)
        rospy.Subscriber('sphero_control/speed', Int16, self.callbackSpeed, queue_size=1, buff_size=1)
        rospy.Subscriber('sphero_control/stopRoll', Empty, self.callbackStopRoll, queue_size=1, buff_size=1)


    def callbackRoll(self, data):
        rospy.loginfo(f'rolling message: {data}')
        self.bolt.roll(min(data.heading, 255), min(data.speed, 255), data.duration)

    def callbackStopRoll(self, data):
        rospy.loginfo(f'stop')
        self.speed = 0
        self.bolt.stop_roll()


    def callbackHeading(self, data):
        rospy.loginfo(f'heading message. {data}')
        newHeading = min(data.data, 255)
        if newHeading == self.heading:
            return
        else:
            self.heading = newHeading
            self.bolt.set_heading(newHeading)

    def callbackSpeed(self, data):
        rospy.loginfo(f'speed message. {data}')
        newSpeed = min(data.data, 255)
        if newSpeed == self.speed:
            return
        else:
            self.speed = newSpeed
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