#!/usr/bin/env python
import rospy
from cse571_project.srv import *
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from server import *
import time
import numpy as np


def spawner():
    rospy.wait_for_service('spawn_book')
    rospy.init_node("spawnner",anonymous=True)
    try:
        print "Press Ctrl+c to quit"
        while(True):
            print "Sleep for 150 secs..."
            rospy.sleep(150)
            print "Spawnning..."
            spawn = rospy.ServiceProxy('spawn_book', SpawnMsg)
            spawn()
    except rospy.ServiceException,e:
        print "Spawnning failed",e


if __name__ == '__main__':
    spawner()
