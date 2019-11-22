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
        while(True):
            print "Spawnning..."
            spawn = rospy.ServiceProxy('spawn_book', SpawnMsg)
            spawn()
            rospy.Rate(1/2.0).sleep()
    except rospy.ServiceException,e:
        print "Spawnning failed",e


if __name__ == '__main__':
    spawner()