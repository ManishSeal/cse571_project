#!/usr/bin/env python

from cse571_project.srv import *
import rospy
from mazeGenerator import *
import sys
import argparse
import time
from action_server import RobotActionsServer
import pickle
import copy
import time
import json
import os
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import pdb

root_path = os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.path.pardir))
books = None
headless = None
mazeInfo = None
mazeInfoCopy = None
parser = argparse.ArgumentParser()
parser.add_argument('-sub', help='for providing no. of subjects',
                    metavar='5', action='store', dest='n_subjects', default=2, type=int)
parser.add_argument('-b', help='for providing no. of books for each subject',
                    metavar='5', action='store', dest='n_books', default=2, type=int)
parser.add_argument('-s', help='for providing random seed', metavar='32',
                    action='store', dest='seed', default=int(time.time()), type=int)
parser.add_argument('-action_seed', help='for providing action selection random seed',
                    metavar='32', action='store', dest='action_seed', default=int(time.time()), type=int)
parser.add_argument('-headless', help='1 to run in the headless mode, 0 to launch gazebo', metavar='1', action='store', dest='headless', default=1, type=int)

robot_action_server = None



def book_dict_generator(bookname,location, coord1, coord2, coord3, coord4):
    global books
    subjects = set()
    for k, v in books['books'].items():
        # print "v: ", v
        # print "type(v): ", type(v)
        if 'subject' in v:
            subjects.add(v["subject"])
        
    # print(bookCounter)
    books['books'][bookname]["size"] = np.random.choice(["small", "large"])
    # Total book count of a subject including small and large
    books['books'][bookname]["subject"] = np.random.choice(list(subjects))
    books['books'][bookname]["loc"]= location
    books['books'][bookname]["load_loc"] = []

    if (coord1[0] > 0 and coord1[1] > 0):
        books['books'][bookname]["load_loc"].append(coord1)

    if (coord2[0] > 0 and coord2[1] > 0):
        books['books'][bookname]["load_loc"].append(coord2)

    if (coord3[0] > 0 and coord3[1] > 0):
        books['books'][bookname]["load_loc"].append(coord3)

    if (coord4[0] > 0 and coord4[1] > 0):
        books['books'][bookname]["load_loc"].append(coord4)

    books['books'][bookname]["placed"] = False


def check_is_edge(req):
    """
    This function checks if two points are connected via edge or not.
    """
    global mazeInfo
    edge = (req.x1, req.y1, req.x2, req.y2)
    for edge_point in edge:
        if edge_point < mazeInfo.grid_start or edge_point > mazeInfo.grid_dimension * 0.5:
            return 0
    if edge in mazeInfo.blocked_edges or (edge[2], edge[3], edge[0], edge[1]) in mazeInfo.blocked_edges:
        return 0
    else:
        return 1


def handle_get_successor(req):
    """
        This function returns successor of a given state.

        parameters:	x_cord - current x-cordinate of turtlebot           output:   x_cord - new x-cordinate of turtlebot
                    y_cord - current y-cordinate of turtlebot					  y_cord - new y-cordinate of turtlebot
                    direction - current orientation								  direction - new orientation
                    action - current action										  g_cost - Manhatan distance from initial state to new state
                                                                                    hurestic_value - Manhatan distance from goal state to new state
    """
    global mazeInfo
    directionList = ["NORTH", "EAST", "SOUTH", "WEST"]
    x_cord, y_cord, direction, action = req.x, req.y, req.direction, req.action

    # Checking requested action and making changes in states
    if action == 'TurnCW':
        index = directionList.index(req.direction)
        direction = directionList[(index+1) % 4]
        g_cost = 2

    elif action == 'TurnCCW':
        index = directionList.index(req.direction)
        direction = directionList[(index-1) % 4]
        g_cost = 2

    elif action == 'moveF':
        if direction == "NORTH":
            y_cord += 0.5
        elif direction == "EAST":
            x_cord += 0.5
        elif direction == "SOUTH":
            y_cord -= 0.5
        elif direction == "WEST":
            x_cord -= 0.5
        g_cost = 1

    elif action == 'moveB':
        if direction == "NORTH":
            y_cord -= 0.5
        elif direction == "EAST":
            x_cord -= 0.5
        elif direction == "SOUTH":
            y_cord += 0.5
        elif direction == "WEST":
            x_cord += 0.5
        g_cost = 3

    rospy.wait_for_service('check_is_edge')
    try:
        check_is_edge = rospy.ServiceProxy('check_is_edge', CheckEdge)
        if req.x <= x_cord and req.y <= y_cord:
            # ((req.x, req.y, x_cord, y_cord), "changedValuesLater")
            result = check_is_edge(req.x, req.y, x_cord, y_cord)
            if result.value == 1:
                isValidEdge = True
            else:
                isValidEdge = False
        else:
            # ((x_cord, y_cord, req.x, req.y), "changedValuesBefore")
            result = check_is_edge(x_cord, y_cord, req.x, req.y)
            if result.value == 1:
                isValidEdge = True
            else:
                isValidEdge = False

    except rospy.ServiceException, e:
        print "Sevice call failed: %s" % e

    if not isValidEdge:
        return GetSuccessorResponse(-1, -1, direction, -1)

    return GetSuccessorResponse(x_cord, y_cord, direction, g_cost)


def handle_reset_world(req):
    global mazeInfo
    mazeInfo = copy.deepcopy(mazeInfoCopy)
    robot_action_server.current_state = robot_action_server.generate_init_state()
    return 1


def remove_blocked_edge(req):
    bookname = req.bookname
    global books
    global mazeInfo
    x, y = books["books"][bookname]["loc"]
    myscale = 0.5
    mazeInfo.blocked_edges.remove((x, y, x+myscale, y))  # V
    mazeInfo.blocked_edges.remove((x, y, x, y+myscale))  # >
    mazeInfo.blocked_edges.remove((x-myscale, y, x, y))  # ^
    mazeInfo.blocked_edges.remove((x, y-myscale, x, y))  # <
    return "1"


def spawn(req):
    global robot_action_server
    global mazeInfo
    global books
    global book_number
    global headless
    print "Headless: " ,headless, type(headless)
    isSimulation = not headless
    if(isSimulation):
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        f = open(root_path+"/helpers/models/book_1/model2.sdf", "r")
        sdff = f.read()
        initial_pose = Pose()
        spawn_model_prox = rospy.ServiceProxy(
            'gazebo/spawn_sdf_model', SpawnModel)
            
    rospy.wait_for_service("update_currentstate_objectdict")
    if book_number < len(books['books']):
        book_number = len(books['books'])
    book_number = book_number + 1
    print "new book number: ", book_number
    bookname = "book_"+str(book_number)  # str(int(time.time()*100))
    myscale = 0.5
    failure = True
    ctr = 0
    
    update_object_prox = rospy.ServiceProxy('update_currentstate_objectdict', UpdatedTuple)
    while failure and ctr < 500:
        x = np.random.randint(0, (mazeInfo.grid_dimension+2)//2)
        y = np.random.randint(0, (mazeInfo.grid_dimension+2)//2)

        #Preventing the book spawning on robot in zero state
        if(x==0 and y == 0):
            ctr+=1
            continue
        if((x <= mazeInfo.grid_dimension*myscale//2)
            and ((x, y, x+myscale, y) not in mazeInfo.blocked_edges)
            and ((x, y, x, y+myscale) not in mazeInfo.blocked_edges)
            and ((x-myscale, y, x, y) not in mazeInfo.blocked_edges)
            and ((x, y-myscale, x, y) not in mazeInfo.blocked_edges)):

            mazeInfo.blocked_edges.add((x, y, x+myscale, y)) # V
            mazeInfo.blocked_edges.add((x, y, x, y+myscale)) # >
            mazeInfo.blocked_edges.add((x-myscale, y, x, y)) # ^
            mazeInfo.blocked_edges.add((x, y-myscale, x, y)) # <
            if(isSimulation):
                initial_pose.position.x = x
                initial_pose.position.y = y
                initial_pose.position.z = 0
                spawn_model_prox(bookname, sdff, bookname, initial_pose, "world")
            
            books["books"][bookname] = {}
            book_dict_generator(bookname, (x, y), (x+myscale, y), (x-myscale, y), (x, y+myscale), (x, y-myscale))
            #print "Book spawned Successfully"
            update_object_prox(json.dumps( [bookname, books["books"][bookname]] ))
            #robot_action_server.update_currentstate_objectdict((bookname,books[bookname]))
            failure = False
            return "Success"

        else:
            failure = True
            ctr +=1

    return "Failure"





def server():
    rospy.Service('remove_blocked_edge',
                  RemoveBlockedEdgeMsg, remove_blocked_edge)
    rospy.Service('check_is_edge', CheckEdge, check_is_edge)
    rospy.Service('reset_world', ResetWorldMsg, handle_reset_world)
    rospy.Service('spawn_book', SpawnMsg, spawn)
    rospy.Service('get_successor', GetSuccessor, handle_get_successor)
    print "Ready!"
    rospy.spin()


if __name__ == "__main__":
    args = parser.parse_args()
    n_subjects = args.n_subjects
    n_books = args.n_books
    seed = args.seed
    headless = args.headless == 1
    if(headless):
        print "Running in headless mode"
    else:
        print "Running in Graphical Interface mode. Launch Gazebo to visualize"
    # print "n_subjectes: ", n_subjects
    # print "n_books:", n_books
    if n_subjects > 20:
        print('Maximum no. of subjects available is: 20')
        exit()
    book_sizes = 2
    book_count_of_each_subject = n_books * book_sizes
    book_count_list = [n_books] * n_subjects * book_sizes
    book_number = len(book_count_list)
    number_of_trollies = n_subjects * 2
    # grid_size = max((((book_count_of_each_subject * n_subjects) / 4) // 1 ) + 1, ((number_of_trollies/4)*7), 10)
    grid_size = 6 * n_subjects
    mazeInfo = Maze(grid_size, 0.5)
    books = mazeInfo.generate_blocked_edges(
        book_count_list, seed,  number_of_trollies, root_path)
    mazeInfoCopy = copy.deepcopy(mazeInfo)
    # print "blocked_edges: "
    # for i in mazeInfo.blocked_edges:
    # print i
    # print "books", books
    rospy.init_node('server')
    robot_action_server = RobotActionsServer(
        books, root_path, headless, args.action_seed)
    server()
    
