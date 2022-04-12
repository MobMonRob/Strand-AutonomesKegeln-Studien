#!/usr/bin/env python

import rospy
import time

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color
from sphero_bolt_node.msg import Roll
from std_msgs.msg import  Int16, Empty
#import ros standard message int 

heading = 0
speed = 0

def callbackRoll(data, bolt):
    rospy.loginfo(f'rolling message: {data}')
    bolt.roll(data.heading, data.speed, data.duration)

def callbackStopRoll(data, bolt):
    rospy.loginfo(f'stop')
    speed = 0
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

def main():
    rospy.loginfo("Connecting to Bolt...")

    toy = scanner.find_BOLT()


    if toy is not None:
        print("connected")
        with SpheroEduAPI(toy) as bolt:
            #bolt = SpheroEduAPI(toy)
            initialize_bolt(bolt)
            listener(bolt)
        


if __name__ == '__main__':
    main()