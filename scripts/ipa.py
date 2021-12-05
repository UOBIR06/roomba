#!/usr/bin/python3

"""
@author ir06
@date 23/11/2021
"""
import sys
import os
import actionlib
import rospy
import rospkg

from ipa_building_msgs.msg import MapSegmentationAction, MapSegmentationGoal, MapSegmentationResult, \
    RoomExplorationAction, RoomExplorationGoal, RoomExplorationResult
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image as SensorImage
from roomba import util


class RoomIPA(object):
    def __init__(self):
        # define publishers
        self._pub_seg = rospy.Publisher('/roomba/segmented_map', MapSegmentationResult, queue_size=0)
        self._pub_exp = rospy.Publisher('/roomba/explored_map', RoomExplorationResult, queue_size=0)

        # Get an action client
        self._sac_seg = actionlib.SimpleActionClient('/room_segmentation/room_segmentation_server',
                                                     MapSegmentationAction)
        self._sac_exp = actionlib.SimpleActionClient('/room_exploration/room_exploration_server',
                                                     RoomExplorationAction)

        rospy.loginfo('[RoomIPA]Waiting for action server(s)...')
        self._sac_seg.wait_for_server()
        self._sac_exp.wait_for_server()

        rospy.loginfo('[RoomIPA]Waiting for map(s)...')
        map_topic = rospy.get_param('map_topic', '/roomba/map_ready')
        expo_topic = rospy.get_param('expo_topic', '/roomba/sweep_grid')
        self._sub_map = rospy.Subscriber(map_topic, OccupancyGrid, self.send_goal_to_segemantation)
        self._sub_expo = rospy.Subscriber(expo_topic, SensorImage, self.send_goal_to_exploration)

    def send_goal_to_segemantation(self, msg: OccupancyGrid) -> MapSegmentationResult:
        rospy.loginfo("Map received. %d X %d." % (msg.info.width, msg.info.height))
        img = util.grid_to_sensor_image(msg)

        # Define the goal
        goal = MapSegmentationGoal()
        goal.input_map = img
        goal.map_resolution = msg.info.resolution
        goal.map_origin = msg.info.origin
        goal.return_format_in_pixel = True
        goal.return_format_in_meter = True
        goal.robot_radius = 0.22  # Same as footprint
        goal.room_segmentation_algorithm = 3

        rospy.loginfo("Waiting for segmentation reply...")
        self._sac_seg.send_goal_and_wait(goal)
        res = self._sac_seg.get_result()
        self._pub_seg.publish(res)
        return res

    def send_goal_to_exploration(self, msg: SensorImage) -> RoomExplorationResult:
        # Define the goal
        goal = RoomExplorationGoal()
        goal.input_map = msg
        goal.map_origin.position.x = 0
        goal.map_origin.position.y = 0
        goal.map_resolution = 0.05
        goal.robot_radius = 0.5
        goal.coverage_radius = 0.5
        goal.starting_position = Pose2D()

        rospy.loginfo("waiting for exploration reply")
        self._sac_exp.send_goal_and_wait(goal)
        res: RoomExplorationResult = self._sac_exp.get_result()
        rospy.loginfo("Got a path with " + str(len(res.coverage_path)) + " nodes.")
        self._pub_exp.publish(res)
        print(res.coverage_path)
        return res



if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("roomba")
    node = RoomIPA()
    rospy.spin()
