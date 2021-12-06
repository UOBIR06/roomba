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
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image as SensorImage
from ipa_building_msgs.msg import MapSegmentationResult, RoomExplorationResult, RoomInformation
from roomba import util
from roomba.classes import RoomInfo


class MDP(object):
    def __init__(self):

        # segmentation related
        self.rooms = dict()  # room centres for each element, key corresponding with room number, starting from 1 !!

        # Do sweeping
        self._pub_sweep = rospy.Publisher('/roomba/sweep_grid', SensorImage, queue_size=20)

        # Get initial map
        grid = rospy.wait_for_message("/map", OccupancyGrid)  # TODO: Change this to '/roomba/map_ready' later
        # self.map_info = grid.info

        # Do segmentation
        seg_map = rospy.wait_for_message("/roomba/segmented_map", MapSegmentationResult)
        self.get_segmented_map(seg_map)
        # self._sub_segmented_map = rospy.Subscriber('/roomba/segmented_map', MapSegmentationResult,
        #                                            self.get_segmented_map)

        # # TODO test data
        # self.rooms[1] = RoomInfo()
        # self.rooms[2] = RoomInfo()
        # self.rooms[3] = RoomInfo()
        # # TODO test data

        self.charging_point = Point32()  # where battery is can be initial point?
        self.area_rate = 100.0  # TODO find some proper number
        self.charge_rate = 1.0  # TODO find some proper number
        self.dis_rate = 1.0  # TODO find some proper number
        self.gamma = 0.8  # TODO: Tweak this
        self.battery_loss_rate = 0.01  # TODO: Tweak this

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

        self.states = self.gen_state()
        # Perhaps simpler to just use a number i.e self.actions = self.num_rooms
        self.actions = [0, 1, 2]  # 0move 1clean,2charge
        self.transitions = {
            0: self.gen_trans_move(),
            # 1: self.gen_trans_clean(),
            # 2: self.gen_trans_charge()
        }
        self.rewards = {
            0: self.gen_reward_move(),
            # 1: self.gen_reward_clean(),
            # 2: self.gen_reward_charge()
        }

        (self.transitions[1], self.rewards[1]) = self.gen_trans_clean()
        (self.transitions[2], self.rewards[2]) = self.gen_trans_charge()

        self.policy_iteration()


    def policy_iteration(self):
        # See RL book pp. 80, section 4.3

        unstable = True
        while unstable:  # Keep going until we reach a stable policy

            # Policy Evaluation
            delta = math.inf
            while delta >= 0.001:
                for s in self.states:  # Iterating over keys (states)
                    a = self.policy.get(s, random.choice(self.actions))  # Policy action (or random)
                    old_v = self.values.get(s, 0)  # Initially zero (pp. 75)
                    new_v = self.bellman(s, a)
                    self.values[s] = new_v

                    # delta <-- max(delta, |v - V(s)|)
                    delta = max(delta, abs(old_v - new_v))

            # Policy Improvement
            unstable = False
            for s in self.states:
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

    def gen_state(self):  # -> tuple:
        # Generates all possible states on request.
        n = len(self.rooms)
        s = []
        for p in range(n + 1):  # Robot pose [-1charging, 0-n-1:rooms]
            for b in range(1):  # FIXME delete battery level for now. Battery level 0low, 1high, -1fucked,[0, 100]
                for i in range(int(math.pow(2, n))):  # clean & unclean
                    bi = "{0:b}".format(i).zfill(n)
                    s.append((bi, p - 1, b))
        return s

    def gen_trans_clean(self):
        t = dict()
        r = dict()
        for i in self.states:  # s
            pos = i[1]
            if pos == -1 or i[0][pos] != '0':
                continue
            t[i] = dict()
            r[i] = dict()
            t[i][(i[0][:pos] + '1' + i[0][pos + 1:], i[1], i[2])] = 1
            r[i][(i[0][:pos] + '1' + i[0][pos + 1:], i[1], i[2])] = self.rooms[pos+1].area * self.area_rate
        return t, r

    def gen_trans_move(self):
        t = dict()
        for i in self.states:  # s
            pos = i[1]
            if pos == -1:
                t[i] = dict()
                # charging to uncleaned
                for ind, j in enumerate(i[0]):
                    if j == '0':
                        t[i][(i[0], ind, i[2])] = 1
            else:
                if i[0][pos] == '0':
                    continue
                t[i] = dict()

                # can go to uncleaned place
                for ind, j in enumerate(i[0]):
                    if j == '0':
                        t[i][(i[0], ind, i[2])] = 1

                # can go to charging place
                t[i][(i[0], -1, i[2])] = 1
            t[i] = util.normalise_dict(t[i])
        return t

    def gen_trans_charge(self):
        t = dict()
        r = dict()
        for i in self.states:  # s
            pos = i[1]
            if pos == -1:
                t[i] = dict()
                r[i] = dict()
                t[i][i] = 1
                r[i][i] = self.charge_rate
        return t, r

    def gen_reward_move(self):
        r = dict()

        for i, s_p in self.transitions[0].items():  # 0move
            r[i] = dict()
            for j in s_p.keys():
                r[i][j] = -self.distance_between_rooms(i[1] + 1, j[1] + 1) * self.dis_rate
        return r

    def gen_reward_clean(self):
        pass

    def gen_reward_charge(self):
        pass

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
        c1 = self.rooms.get(r1).centre if r1 > 0 else self.charging_point
        c2 = self.rooms.get(r2).centre if r2 > 0 else self.charging_point

        dis = np.linalg.norm(np.array([c1.x, c1.y, c1.z]) - np.array([c2.x, c2.y, c2.z]))
        return dis

    def distance_to_battery(self, d: float) -> int:  # [0, 100]
        n = d / self.map_info.resolution  # Number of cells traveled
        return min(100, n * self.battery_loss_rate)  # TODO: Not sure if this is good

    def get_segmented_map(self, result: MapSegmentationResult) -> None:  # see self.rooms
        self.rooms = dict()
        li = [x if 0 < x <= len(result.room_information_in_meter) else 0 for x in result.segmented_map.data]
        total_map_area = np.count_nonzero(li)
        for idx, room in enumerate(result.room_information_in_meter):
            info = RoomInfo()
            info.id = idx + 1
            # Remember: free space (0), unknown (-1), obstacle (100)
            data = [0 if x == info.id else 255 for x in li]
            info.area = (len(data) - np.count_nonzero(data)) / total_map_area
            if info.area < 0.001:  # too small
                continue
            info.centre = room.room_center
            info.img = util.gen_sensor_img_with_data(data, result.segmented_map)
            self.rooms[info.id] = info
        rospy.loginfo('[mdp]got %d rooms.' % len(self.rooms))

        for i in self.rooms.values():
            self._pub_sweep.publish(i.img)

        # self.policy_iteration()

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
