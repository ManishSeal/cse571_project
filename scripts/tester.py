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

helper = problem.Helper()

unplaced_books = helper.get_unplaced_books()

current_state = helper.get_current_state()

root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))

books_json_file = root_path + "/books.json"

with open(books_json_file, "r") as fp:
    init_objects_dict = json.load(fp)
    
print "init_objects_dict: ", init_objects_dict



print "unplaced_books: \n", unplaced_books
print "type: ", type(unplaced_books)

print "current_state: ", current_state

subject_size_bin_dict = []
for trolly, desc in init_objects_dict['bins'].items():
    size = desc['size']
    subject = desc['subject']
    if subject not in subject_size_bin_dict:
        subject_size_bin_dict[subject] = dict():
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
        



    

