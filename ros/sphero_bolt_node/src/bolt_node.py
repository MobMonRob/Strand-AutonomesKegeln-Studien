#!/usr/bin/env python

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
    bolt.stop()


def callbackHeading(data, bolt):
    rospy.loginfo(f'heading message. {data}')
    bolt.heading(data)

def callbackSpeed(data, bolt):
    rospy.loginfo(f'speed message. {data}')
    bolt.speed(data)


def listener(bolt):

# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
    rospy.init_node('sphero', anonymous=True)
    rospy.loginfo('listener node is up')
    rospy.Subscriber('sphero_control/roll', Roll, callbackRoll, bolt)
    rospy.Subscriber('sphero_control/heading', Int16, callbackHeading, bolt)
    rospy.Subscriber('sphero_control/speed', Int16, callbackSpeed, bolt)
    rospy.Subscriber('sphero_control/stopRoll', Empty, callbackStopRoll, bolt)

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