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

object_dict = helper.get_unplaced_books()

# current_state = helper.get_current_state()

# root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))

# books_json_file = root_path + "/books.json"

# with open(books_json_file, "r") as fp:
#     init_objects_dict = json.load(fp)
    
# print "init_objects_dict: ", init_objects_dict



print "object_dict: \n", object_dict
print "type: ", type(object_dict)

# print "current_state: ", current_state



def get_manhattan_distance(start_loc, end_loc):
    """
    Returns the manhattan distance between 2 states
    
    """
    start_loc = np.array(start_loc)
    end_loc = np.array(end_loc)
    return sum(abs(start_loc - end_loc))

def get_open_load_loc(load_locs, occupied_locs):
    for loc_x, loc_y in load_locs:
        if (loc_x, loc_y) not in occupied_locs:
            return (loc_x, loc_y)
    return (-1, -1)
    
bin_subject_name_dict = {}

current_occupied_locs = set()
age_dict = {}
unplaced_books = set()
for trolly, desc in object_dict['bins'].items():
    loc_x, loc_y = desc['loc']
    current_occupied_locs.add((loc_x, loc_y))
    bin_subject_name_dict[(desc['subject'], desc['size'])] = trolly
    
for book, desc in object_dict['books'].items():
    loc_x, loc_y = desc['loc']
    current_occupied_locs.add((loc_x, loc_y))
    if book not in age_dict:
        age_dict[book] = 0
    else:
        age_dict[book] += 5
    if desc["placed"] is False:
        unplaced_books.add(book)
priority_queue = []

for i in bin_subject_name_dict:
    print i
    
# get robot_current_loc
current_state = helper.get_current_state()
robot_state = current_state['robot']
robot_loc = (robot_state['x'], robot_state['y'])

for book in unplaced_books:
    print book
    book_desc = object_dict["books"][book]
    book_load_locs = book_desc['load_loc']
    open_book_load_loc = get_open_load_loc(book_load_locs, current_occupied_locs)
    if open_book_load_loc == (-1, -1):
        continue
    print open_book_load_loc
    trolly = bin_subject_name_dict[(book_desc['subject'], book_desc['size'])]
    print "trolly: ", trolly
    trolly_desc = object_dict["bins"][trolly]
    trolly_load_locs = trolly_desc['load_loc']
    open_trolly_load_loc = get_open_load_loc(trolly_load_locs, current_occupied_locs)
    if open_trolly_load_loc == (-1, -1):
        continue
    print open_trolly_load_loc
    distance_book_trolly = get_manhattan_distance(open_book_load_loc, open_trolly_load_loc)
    distance_robot_book = get_manhattan_distance(open_book_load_loc, robot_loc)
    age = age_dict[book]
    total_distance = distance_book_trolly + distance_robot_book
    print "total_distance: ",total_distance
    heapq.heappush(priority_queue, (total_distance - age, book, trolly))
    
    
while len(priority_queue) > 0:
    _, book, trolly = heapq.heappop(priority_queue)
    print "book: ", book, "trolly: ", trolly
    if book in unplaced_books:
        break
else:
    print("not found")
    
    
robot = Robot()


print "current_state_robot: ", current_state['robot']
#success, next_state = helper.execute_action("moveF", {})
#print "next_state: ", next_state['robot']

robot = Robot()
init_state = State(current_state['robot']['x'], current_state['robot']['y'], current_state['robot']['orientation'])

path = robot.get_path(init_state, object_dict['books'][book]['load_loc'])

print path



    
    


"""
subject_size_bin_dict = []
for trolly, desc in init_objects_dict['bins'].items():
    size = desc['size']
    subject = desc['subject']
    if subject not in subject_size_bin_dict:
        subject_size_bin_dict[subject] = dict()
    if size not in subject_size_bin_dict[subject]:
        subject_size_bin_dict[subject][size] = {}
    subject_size_bin_dict[subject][size].append(trolly)

load_loc = set()
occupied_loc = set()
for b_dict in unplaced_books:
    b_name = b_dict['name']
    for u,v in b_dict['load_loc']:
        load_loc.add((u,v))
    u, v = b_dict['loc']
    occupied_loc.add((u,v))
    
for b_dict in unplaced_books:
    for bin_dict in init_objects_dict['bins']:
"""

