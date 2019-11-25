#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Abhyudaya Srinet", "Manish Seal"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
from std_msgs.msg import String
import problem
import json
import os
import argparse
import time
import heapq

import numpy as np
from robot import Robot
from problem import State

helper = problem.Helper()

object_dict = {}
bin_subject_name_dict = {}
current_occupied_locs = set()
age_dict = {}
unplaced_books = set()
priority_queue = []
current_state = ()
robot = Robot()



def getLatestObjectDictionary():
    global object_dict
    object_dict = helper.get_unplaced_books()

def get_manhattan_distance(start_loc, end_loc):
    """
    Returns the manhattan distance between 2 states

    """
    start_loc = np.array(start_loc)
    end_loc = np.array(end_loc)
    return sum(abs(start_loc - end_loc))


def get_open_load_loc(load_locs, occupied_locs):
    for loc_x, loc_y in load_locs:
        if(len(occupied_locs) == 0):
            return (loc_x, loc_y)
        if (loc_x, loc_y) not in occupied_locs:
            return (loc_x, loc_y)
    return (-1, -1)


def updateGetBinsAndOccupiedLocations():
    global bin_subject_name_dict
    global current_occupied_locs
    global object_dict
    for trolly, desc in object_dict['bins'].items():
        loc_x, loc_y = desc['loc']
        current_occupied_locs.add((loc_x, loc_y))
        bin_subject_name_dict[(desc['subject'], desc['size'])] = trolly


def updateUnplacedBooksAndLocations():
    global age_dict
    global current_occupied_locs
    global unplaced_books
    global object_dict
    for book, desc in object_dict['books'].items():
        if desc["placed"] is False:
            if book not in age_dict:
                age_dict[book] = 0
            else:
                age_dict[book] += 5
            if book not in unplaced_books:
                unplaced_books.add(book)
                loc_x, loc_y = desc['loc']
                current_occupied_locs.add((loc_x, loc_y))

def getCurrentRobotLocation():
    global current_state
    current_state = helper.get_current_state()
    robot_state = current_state['robot']
    return (robot_state['x'], robot_state['y'])

def updatePriorityQueue():
    global unplaced_books
    global current_occupied_locs
    global object_dict
    global bin_subject_name_dict
    for book in unplaced_books:
        book_desc = object_dict["books"][book]
        book_load_locs = book_desc['load_loc']
        open_book_load_loc = get_open_load_loc(
            book_load_locs, current_occupied_locs)
        if open_book_load_loc == (-1, -1):
            continue
        trolly = bin_subject_name_dict[(book_desc['subject'], book_desc['size'])]
        trolly_desc = object_dict["bins"][trolly]
        trolly_load_locs = trolly_desc['load_loc']
        open_trolly_load_loc = get_open_load_loc(
            trolly_load_locs, current_occupied_locs)
        if open_trolly_load_loc == (-1, -1):
            continue
        distance_book_trolly = get_manhattan_distance(
            open_book_load_loc, open_trolly_load_loc)
        distance_robot_book = get_manhattan_distance(open_book_load_loc, getCurrentRobotLocation())
        age = age_dict[book]
        total_distance = distance_book_trolly + distance_robot_book
        heapq.heappush(priority_queue, (total_distance - age, book, trolly))

def get_next_bookandtrolly():
    global unplaced_books
    global object_dict
    while len(priority_queue) > 0:
        _, book, trolly = heapq.heappop(priority_queue)
        if(len(priority_queue) == 0):
                print("All books are placed in this instance")
        elif book in unplaced_books:# and len(book)>len("book_8"):
            print "Will place book: ", book, "in trolly: ", trolly
            return book, trolly
    return "",""

def getRobotState():
    current_state = helper.get_current_state()
    return State(current_state['robot']['x'], current_state['robot']['y'], current_state['robot']['orientation'])


def carefulPerformTask(path, do="pick", trolly=None):
    global unplaced_books
    success = False
    for action in path[0]:
        print "action: ", action
        success, next_state = helper.execute_action(action, {})
        #print success, next_state
        if success == False:
            print "Old path invalid! Rerouting..."
            path = robot.get_path(getRobotState(), object_dict['books'][book]['load_loc'])
            if(path == []):
                print book, "No valid path avaialable!"
                return False
                break
            return carefulPerformTask(path)
            break
    params = {}
    if do == 'pick':
        params["book_name"] = book
        success, next_state = helper.execute_action('pick', params)
        if(not success):
            print "Pick action failed"
    else:
        params["book_name"] = book
        params["bin_name"] = trolly
        success, next_state = helper.execute_action('place', params)
        if(not success):
            print "Place action failed"
        else:
            unplaced_books.remove(book)
    return success


def PerformOneIteration(robot_state, book, trolly):
    global current_occupied_locs
    path = robot.get_path(robot_state, object_dict['books'][book]['load_loc'])
    print "Path from robot to book: ", path
    reached_loc_and_picked = carefulPerformTask(path, 'pick')
    if reached_loc_and_picked:
        print book, " reached and picked"
        book_locx,book_locy = object_dict['books'][book]['loc']
        current_occupied_locs.remove((book_locx, book_locy))
        path = robot.get_path(getRobotState(), object_dict["bins"][trolly]['load_loc'])
        print "Path from book to trolly: ", path 
        reached_loc_and_placed = carefulPerformTask(path, 'place', trolly)
        if not reached_loc_and_placed:
            print book, " could not be delivered to ", trolly
            return False
        else:
            print book, " delivered to ", trolly
            return True
    return False


if __name__ == "__main__":
    getLatestObjectDictionary()
    updateGetBinsAndOccupiedLocations()
    while(True):
        getLatestObjectDictionary()
        updateUnplacedBooksAndLocations()
        updatePriorityQueue()
        book, trolly = get_next_bookandtrolly()
        if(book==""):
            print "No books lying around. Will sleep for 150 seconds and check again ;)."
            print "Press Ctrl+c to Quit"
            rospy.sleep(150)
            continue
        robot_state = getRobotState()
        PerformOneIteration(robot_state, book, trolly)
        print("One Iteration Done")
