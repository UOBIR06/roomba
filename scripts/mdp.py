#!/usr/bin/python3

import os
import struct
import sys
import rospkg
import rospy
import math
import random
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from ipa_building_msgs.msg import MapSegmentationResult, MapSegmentationGoal, RoomExplorationResult
from room_ipa import RoomIPA


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
        self.battery = 100 # battery level 0-100

        # TODO: Populate according to # of rooms + recharge
        #   -1 : recharge
        #   [0, inf] : clean room x
        # Perhaps simpler to just use a number i.e self.actions = self.num_rooms
        self.actions = []
        self.gamma = 0.8  # TODO: Tweak this
        self.battery_loss_rate = 0.01   # TODO: Tweak this
        #self._room_ipa = RoomIPA()
        #self.get_segmented_map(img_path='')
        #self.do_sweeping(img_path='')

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
        # Pseudo transition function replacement.
        #
        # Returns the end states, s', which are possible
        # to reach starting from state, s, taking action, a.
        # We do this to avoid iterating over the huge state
        # space, because most of them don't make sense:
        #   e.g. can't go from a room being clean to unclean,
        #   when the action taken was to go recharge!
        pass

    def reward(self, s, a, p) -> int:
        reward = 0
        # penalise actions which would leave the robot stranded
        if not next.canReturntoCharge():
            reward -= math.inf
            # incentivise more clean rooms
            if next.noOfCleanRooms > s.noOfCleanRooms:
                reward += 50
            # add battery level to reward, seek to conserve bettery
            reward += next[-1]
            # penalise outcomes which result in the robot moving further away from its current state
            reward -= self.distance_between_states(self, state[-2], next[-2])
            return reward

    def noOfCleanrooms(s):
        cleanrooms = 0
        for i in range(0, len(s) - 2):
            cleanrooms += s[i]
        return cleanrooms

    # @Yanrong will write it.
    def get_estimate_battery_left(self, room_in_now:int, room_to_go:int) -> int: # battery level 0-100
        # including bettery used for clean room 'room_to_go' and battery used for go to there
        # if you don't want the distance included, just pass room_in_now as room_to_go
        # will not update self.battery. just do the estimation.
        pass

    def state_to_area(self, s) -> float:  # areas or distance inside room s
        pass

    def distance_between_states(self, s1, s2) -> float:  # distance
        pass

    def distance_to_battery(self, s, s_p) -> int:  # battery level 0-100
        pass

    def get_init_map(self) -> OccupancyGrid:
        pass

    def get_segmented_map(self, img_path: str) -> MapSegmentationResult:  # list of [{centre, area, id}]
        # TODO read from OccupancyGrid instead of png file
        # img_path = os.path.join(rospkg.RosPack().get_path('roomba'), 'data/sim_data/meeting.png')
        res: MapSegmentationResult = self._room_ipa.send_goal_to_segemantation(img_path=img_path)
        return res

    def do_sweeping(self, img_path: str) -> RoomExplorationResult:
        # TODO use segments from get_segmented_map as inputs
        res: RoomExplorationResult = self._room_ipa.send_goal_to_exploration(img_path=img_path)
        return res


if __name__ == '__main__':
    rospy.init_node('mdp')

    # Grab explored map when ready
    msg = rospy.wait_for_message('/map', OccupancyGrid)

    # Convert it to an image
    data = bytes(map(lambda x: 255 if x == 0 else 0, msg.data))

    # Stuff it inside a sensor_msgs/Image
    img = Image()
    img.width = msg.info.width
    img.height = msg.info.height
    img.encoding = '8UC1'
    img.is_bigendian = (sys.byteorder == 'big')
    img.step = img.width
    img.data = data

    # Now stuff the image inside a MapSegmentationGoal
    goal = MapSegmentationGoal()
    goal.input_map = img
    goal.map_resolution = msg.info.resolution
    goal.map_origin = msg.info.origin
    goal.return_format_in_pixel = False
    goal.return_format_in_meter = True
    goal.robot_radius = 0.22  # Same as footprint
    goal.room_segmentation_algorithm = 0

    # Call segmentation server
    client = RoomIPA()  # TODO: Yanrong, can you use the code above?
    reply = client.send_goal_to_segemantation(goal)

    mdp = MDP()
    rospy.spin()
