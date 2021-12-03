#!/usr/bin/python3

import os
import struct
import sys

import room_ipa
import rospy
import math
import random
from nav_msgs.msg import OccupancyGrid
from ipa_building_msgs.msg import MapSegmentationResult, MapSegmentationGoal, RoomExplorationResult
from room_ipa import util


class MDP(object):
    def __init__(self):
        # States are tuples: (r0, ..., rn, pose, battery)
        #   r0, ..., rn : cleanliness state of room i
        #   pose : which room the robot is in, [0, n]
        #   battery : whole number, [0, 100]
        #
        # The layout is like this, so it's easy to get:
        #   pose = state[-2]
        #   battery = state[-1]
        #   ri = state[i]
        self.values = {}
        self.policy = {}
        self.battery = 100  # Battery level [0, 100]

        # TODO: Populate according to # of rooms + recharge
        #   -1 : recharge
        #   [0, inf] : clean room x
        # Perhaps simpler to just use a number i.e self.actions = self.num_rooms
        self.actions = []
        self.gamma = 0.8  # TODO: Tweak this
        self.battery_loss_rate = 0.01  # TODO: Tweak this
        self.rooms = []

        # Get initial map
        grid = rospy.wait_for_message("/map", OccupancyGrid)  # TODO: Change this to '/roomba/map_ready' later
        self.map_info = grid.info

        # Do segmentation
        self._room_ipa = room_ipa.RoomIPA()
        segments = self._room_ipa.send_goal_to_segemantation(grid)

    def policy_iteration(self):
        # See RL book pp. 80, section 4.3

        unstable = True
        while unstable:  # Keep going until we reach a stable policy

            # Policy Evaluation
            delta = math.inf
            while delta >= 0.001:
                for s in self.values:  # Iterating over keys (states)
                    a = self.policy.get(s, random.choice(self.actions))  # Policy action (or random)
                    old_v = self.values.get(s, 0)  # Initially zero (pp. 75)
                    new_v = self.bellman(s, a)
                    self.values[s] = new_v

                    # delta <-- max(delta, |v - V(s)|)
                    delta = max(delta, abs(old_v - new_v))

            # Policy Improvement
            unstable = False
            for s in self.values:
                old_a = self.policy.get(s, random.choice(self.actions))
                new_a = None
                max_v = 0

                # pi(s) <-- argmax_a sum_{s'} p(s' | s, a) [ r(s, a, s') + gamma * V(s') ]
                for a in self.actions:
                    v = self.bellman(s, a)
                    if v > max_v:
                        max_v = v
                        new_a = a
                self.policy[s] = new_a

                if old_a != new_a:
                    unstable = True

    def bellman(self, s, a) -> float:
        # Calculate part of the bellman equation for a state and current policy
        # V(s) = sum_{s'} T(s' | s, a) [ r(s, a, s') + gamma * V(s') ]
        # We forgo T(...) using `self.end_states` here.
        return sum([
            self.reward(s, a, e) + self.gamma * self.values.get(e, 0)
            for e in self.end_states(s, a)
        ])

    def end_states(self, s, a) -> list:
        possible_states = []
        for state in self.values:
            #In the case that action is to recharge..
            if a == -1:
                #Check that battery level of new state is greater than current
                if (s[-1] < state[-1]):
                    valid = True
                    #check that the clean status of each room is exactly the same
                    for i in range (0, len(s)-2):
                        if s[i] != state[i]:
                                #if any difference, valid flag raised
                                valid = False
                    if valid == True:
                        possible_states.append(state)


            # if robot is cleaning a room, only states with the 
            # expected battery level after cleaning would be valid
            elif (s[-1]  - self.battery_to_clean(a)) == state[-1]:
                #check that each room apart from the one being cleaned 
                #isn't in a different state of cleanliness
                valid = True
                for i in range (0, len(s)-2):
                    if s[i] != state[i]:
                        if i != a:
                            valid = False
                if valid == True:
                    possible_states.append(state)

    #reward function, determines how much the robot will value a given action
    def reward(self, s, a, p) -> int:
        reward = 0
        # penalise actions which would leave the robot stranded
        if self.get_estimate_battery_left(s[-2], p[-2]) < self.distance_to_battery(s, self.battery_location):
            reward -= math.inf
        # incentivise more clean rooms
        if self.noOfCleanRooms(p) > self.noOfCleanRooms(s):
            reward += 50
        # add battery level to reward, seek to conserve bettery
        reward += p[-1]
        # penalise outcomes which result in the robot moving further away from its current state
        reward -= self.distance_between_states(self, s[-2], p[-2])
        return reward

    def count_clean_rooms(self, s) -> int:
        return sum(s[:-2])  # @Jacob: This was noOfCleanRooms

    # @Yanrong will write it.
    def get_estimate_battery_left(self, room_in_now: int, room_to_go: int) -> int:  # [0, 100]
        # including battery used for clean room 'room_to_go' and battery used to go there
        # if you don't want the distance included, just pass room_in_now as room_to_go
        # will not update self.battery. just do the estimation.
        pass

    def distance_between_rooms(self, r1: int, r2: int) -> float:
        pass

    def distance_to_battery(self, d: float) -> int:  # [0, 100]
        n = d / self.map_info.resolution  # Number of cells traveled
        return max(100, n * self.battery_loss_rate)  # TODO: Not sure if this is good

    def do_sweeping(self, img_path: str) -> RoomExplorationResult:
        # TODO use segments from get_segmented_map as inputs
        res: RoomExplorationResult = self._room_ipa.send_goal_to_exploration(img_path=img_path)
        return res


if __name__ == '__main__':
    rospy.init_node('mdp')
    mdp = MDP()
    rospy.spin()
