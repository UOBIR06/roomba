#!/usr/bin/python3

import os
import struct
import sys
import rospy
import math
import cv_bridge
import random
import numpy as np
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import Image
from ipa_building_msgs.msg import MapSegmentationResult, RoomExplorationResult, RoomInformation
from roomba import util
from roomba.classes import RoomInfo
from sensor_msgs.msg import Image as SensorImage
from copy import deepcopy
# import mock


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
        self.segmented_map = None
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
        # segmentation related
        self.segmented_map_last_id = -1  # TODO check if needed
        self.segmented_map: MapSegmentationResult  # of type MapSegmentationResult
        self.rooms = dict()  # room centres for each element, index corresponding with room number, starting from 1 !!

        # Get initial map
        grid = rospy.wait_for_message("/map", OccupancyGrid)  # TODO: Change this to '/roomba/map_ready' later
        self.map_info = grid.info

        # Do segmentation
        self._sub_segmented_map = rospy.Subscriber('/roomba/segmented_map', MapSegmentationResult,
                                                   self.get_segmented_map)
        # self._sub_segmented_map = rospy.Subscriber('/room_segmentation/room_segmentation_server/segmented_map',
        #                                            OccupancyGrid, self.get_segmented_grid)

        # Do sweeping
        self._pub_sweep = rospy.Publisher('/roomba/sweep_grid', OccupancyGrid, queue_size=20)


    def policy_iteration(self):
        # See RL book pp. 80, section 4.3

        unstable = True
        while unstable:  # Keep going until we reach a stable policy

            # Policy Evaluation
            delta = math.inf
            while delta >= 0.001:
                for s in self.gen_state():  # Iterating over keys (states)
                    a = self.policy.get(s, random.choice(self.actions))  # Policy action (or random)
                    old_v = self.values.get(s, 0)  # Initially zero (pp. 75)
                    new_v = self.bellman(s, a)
                    self.values[s] = new_v

                    # delta <-- max(delta, |v - V(s)|)
                    delta = max(delta, abs(old_v - new_v))

            # Policy Improvement
            unstable = False
            for s in self.gen_state():
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

    def gen_state(self) -> tuple:
        # Generates all possible states on request.
        n = len(self.rooms)
        r = [0 for _ in range(n)]
        for p in range(n):  # Robot pose [0, n)
            for b in range(101):  # Battery level [0, 100]
                while True:
                    carry = True
                    i = 0
                    while i < n and carry:  # Binary addition
                        r[i] = (r[i] + 1) % 2
                        carry = r[i] == 0
                        i += 1
                    yield *r, p, b
                    if not any(r):
                        break  # Stop when we loop around to (0, 0, ..., 0)

    def bellman(self, s, a) -> float:
        # Calculate part of the bellman equation for a state and current policy
        # V(s) = sum_{s'} T(s' | s, a) [ r(s, a, s') + gamma * V(s') ]
        # We forgo T(...) using `self.end_states` here.
        return sum([
            self.reward(s, a, e) + self.gamma * self.values.get(e, 0)
            for e in self.end_states(s, a)
        ])

    def end_states(self, s, a) -> list:
        # The point of this function is to *avoid* iterating over the enormous
        # state space (i.e. for s in self.values) because we *know* the transitions
        # that are actually possible are a small, predictable fraction of the space.

        cur_room = s[-2]
        cur_batt = s[-1]

        if a == -1:  # Recharge action
            batt = self.get_estimate_battery_left(cur_room, 0)  # NOTE: Assuming room #0 has the charger
        else:  # Clean action
            batt = self.get_estimate_battery_left(cur_room, a)

        diff_batt = cur_batt - batt
        if diff_batt <= 0:
            return []  # Action would strand robot, not possible

        s[a] = 1  # If a == -1 this changes battery which is over-written below, otherwise sets room `a` to clean (1)
        s[-2] = 0 if a == -1 else a  # New room
        s[-1] = diff_batt  # New battery
        return [s]

    # reward function, determines how much the robot will value a given action
    def reward(self, s, a, p) -> int:
        reward = 0
        # penalise actions which would leave the robot stranded
        if self.get_estimate_battery_left(s[-2], p[-2]) < self.distance_to_battery(s, self.battery_location):
            reward -= math.inf
        # incentivise more clean rooms
        if self.count_clean_rooms(p) > self.count_clean_rooms(s):
            reward += 50
        # add battery level to reward, seek to conserve bettery
        reward += p[-1]
        # penalise outcomes which result in the robot moving further away from its current state
        reward -= self.distance_between_rooms(self, s[-2], p[-2])
        return reward

    def count_clean_rooms(self, s) -> int:
        return sum(s[:-2])  # @Jacob: This was noOfCleanRooms
        # sum([i.cleaned for i in self.rooms])

    def get_estimate_battery_left(self, room_to_go: int, room_in_now: int) -> int:  # [0, 100]
        # including battery used for clean room 'room_to_go' and battery used to go there
        # if you don't want the distance included, just pass room_in_now as room_to_go
        # will not update self.battery. just do the estimation.
        r2 = self.rooms.get(room_to_go)
        bat = 0
        bat += r2.area * self.battery_loss_rate  # TODO I think it need to multiply by the whole area of map
        if room_to_go != room_in_now:
            dis = self.distance_between_rooms(room_in_now, room_to_go)
            bat += dis * self.battery_loss_rate
        return self.battery - bat

    def distance_between_rooms(self, r1: int, r2: int) -> float:
        c1 = self.rooms.get(r1).centre
        c2 = self.rooms.get(r2).centre
        dis = np.linalg.norm(np.array([c1.x, c1.y, c1.z]) - np.array([c2.x, c2.y, c2.z]))
        return dis

    def distance_to_battery(self, d: float) -> int:  # [0, 100]
        n = d / self.map_info.resolution  # Number of cells traveled
        return max(100, n * self.battery_loss_rate)  # TODO: Not sure if this is good

    def get_segmented_grid(self, res: OccupancyGrid):
        # total_map_area = len(np.nonzero(res))
        # s = np.array( res.data)
        # x=np.nonzero(res.data)
        # t=0
        # for i in (res.data):
        #     if i:
        #         t+=1
        # print(total_map_area)
        pass

    def get_segmented_map(self, result: MapSegmentationResult) -> None:  # see self.rooms
        # if result.segmented_map.header.stamp != self.segmented_map_last_id:
        #     self.segmented_map_last_id = result.segmented_map.header.stamp
        self.segmented_map = result
        self.rooms = dict()

        # FIXME Here's where I have trouble with
        # li = deepcopy() #, byteorder=sys.byteorder)
        # li = mock.a
        li = [x for x in self.segmented_map.segmented_map.data]
        total_map_area = len(np.nonzero(li)[0])
        for idx, room in enumerate(result.room_information_in_meter):
            info = RoomInfo()
            info.id = idx + 1
            info.centre = room.room_center
            # Remember: free space (0), unknown (-1), obstacle (100)
            data = [0 if x == info.id else 255 for x in li]
            info.img = util.gen_sensor_img_with_data(data, self.segmented_map.segmented_map)
            info.area = len(np.nonzero(data)[0]) / total_map_area
            self.rooms[info.id] = info
        print(len(self.rooms))

    def do_sweeping(self, room_number: int) -> None:
        current_room = self.rooms.get(room_number)
        self._pub_sweep.publish(current_room.grid)
        cpath = rospy.wait_for_message('/roomba/calc_coverage_path', RoomExplorationResult)

        # TODO drive the robot around
        # @Mert can we use something in move_base?

        self.rooms[room_number].cleaned = 1


if __name__ == '__main__':
    rospy.init_node('mdp')
    mdp = MDP()
    rospy.spin()
