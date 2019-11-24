#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Abhyudaya Srinet", "Pulkit Verma"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
import problem
import heapq
import argparse
import os
import json
from std_msgs.msg import String

ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))

class Robot:

    def __init__(self):
        self.helper = problem.Helper()
    # --------------- HELPER FUNCTIONS --------------- #

    def is_goal_state(self, current_state, goal_state):
        """
        Checks if the current_state is goal_state or not. 
        If you are wondering why we are checking orientation, remember this is a different Homework. :)

        """
        if(current_state.x == goal_state.x and current_state.y == goal_state.y and current_state.orientation == goal_state.orientation):
            return True
        return False


    def get_manhattan_distance(self, from_state, to_state):
        """
        Returns the manhattan distance between 2 states
        
        """
        return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)


    def build_goal_states(self, locations):
        """
        Creates a State representations for given list of locations
        
        """
        states = []
        for location in locations:
            states.append(problem.State(location[0], location[1], "EAST"))
            states.append(problem.State(location[0], location[1], "WEST"))
            states.append(problem.State(location[0], location[1], "NORTH"))
            states.append(problem.State(location[0], location[1], "SOUTH"))
        return states


    def get_path(self, init_state, goal_locations):
        """
        This method searches for a path from init_state to one of the possible goal_locations

        :param init_state: Current state of robot
        :type init_state: State
        :param goal_locations: list of target locations to search the path e.g. [(x1, y1), (x2, y2), ..]. This is important if there are multiple books of a subject.
        :type goal_locations: list(State)

        :returns: 
            .. hlist::
                :columns: 1

                * **action_list**: list of actions to execute to go from source to target
                * **final_state**: target state that is reached (will be one of goal_locations)
                * **goal_reached**: True/False indicating if one of the goal_locations was reached
        
        """
        print "Getting the path"
        final_state = None
        goal_states = self.build_goal_states(goal_locations)
        goal_reached = False
        for goal_state in goal_states: #search for any of the load locations
            possible_actions = self.helper.get_actions()
            action_list = []

            state_queue = []
            heapq.heappush(state_queue, (self.get_manhattan_distance(init_state, goal_state), 0,(init_state, [])))
            visited = []
            state_cost = {}
            insert_order = 0

            while len(state_queue) > 0:
                top_item = heapq.heappop(state_queue)
                current_cost = top_item[0]
                current_state = top_item[2][0]
                current_actions = top_item[2][1]

                if(current_state in visited):
                    continue

                if(self.is_goal_state(current_state, goal_state)):
                    goal_reached = True
                    break

                visited.append(current_state)
                for action in possible_actions:
                    nextstate, cost = self.helper.get_successor(current_state, action)
                    cost = self.get_manhattan_distance(nextstate, goal_state) # manhattan distance heuristc
                    key = (nextstate.x, nextstate.y, nextstate.orientation)
                    if(nextstate.x == -1 and nextstate.y == -1):
                        continue
                    # if a new state is found then add to queue
                    if(nextstate not in visited and key not in state_cost.keys()):
                        heapq.heappush(state_queue, (cost, insert_order, (nextstate, current_actions + [action])))
                        insert_order += 1
                        state_cost[key] = cost

            if(self.is_goal_state(current_state, goal_state)):
                action_list = current_actions
                final_state = current_state
                goal_reached = True
                break

        return action_list, final_state, goal_reached
