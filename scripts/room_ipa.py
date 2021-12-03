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
import dynamic_reconfigure.client
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image as SensorImage
from roomba import util


class RoomIPA(object):
    def __init__(self):
        # Get an action client
        self._sac_seg = actionlib.SimpleActionClient('/room_segmentation/room_segmentation_server',
                                                     MapSegmentationAction)
        self._sac_exp = actionlib.SimpleActionClient('/room_exploration/room_exploration_server',
                                                     RoomExplorationAction)

        rospy.loginfo('[RoomIPA]Waiting for action server(s)...')
        self._sac_seg.wait_for_server()
        self._sac_exp.wait_for_server()

        rospy.loginfo('[RoomIPA]Waiting for map(s)...')
        self.last_map = None
        map_topic = rospy.get_param('map_topic', '/roomba/map_ready')
        self._sub_map = rospy.Subscriber(map_topic, OccupancyGrid, self.map_ready_cb)

    def send_goal_to_segemantation(self, msg: OccupancyGrid) -> MapSegmentationResult:
        img = util.grid_to_sensor_image(msg)

        # Define the goal
        goal = MapSegmentationGoal()
        goal.input_map = img
        goal.map_resolution = msg.info.resolution
        goal.map_origin = msg.info.origin
        goal.return_format_in_pixel = False
        goal.return_format_in_meter = True
        goal.robot_radius = 0.22  # Same as footprint
        goal.room_segmentation_algorithm = 3

        rospy.loginfo("Waiting for segmentation reply...")
        self._sac_seg.send_goal_and_wait(goal)
        return self._sac_seg.get_result()

    # TODO not ready yet
    def send_goal_to_exploration(self, goal: RoomExplorationGoal = None, img_path: str = '') -> RoomExplorationResult:
        if not img_path:
            img_path = os.path.join(rospkg.RosPack().get_path('roomba'), 'data/sim_data/meeting.png')
        map_img: SensorImage = util.setup_image(img_path)

        # testing:
        if not goal:
            # Define the goal
            goal = RoomExplorationGoal()
            goal.input_map = map_img
            goal.map_origin.position.x = 0
            goal.map_origin.position.y = 0
            goal.map_resolution = 0.05
            goal.robot_radius = 0.5
            goal.coverage_radius = 0.5
            goal.starting_position = Pose2D()
            # goal.planning_mode

        rospy.loginfo("waiting for result")
        self._sac_exp.send_goal_and_wait(goal)
        response: RoomExplorationResult = self._sac_exp.get_result()
        rospy.loginfo("Got a path with " + str(len(response.coverage_path)) + " nodes.")
        return response

    def map_ready_cb(self, args: OccupancyGrid):
        print(args)

        if self.last_map and self.last_map.header.frame_id == args.header.frame_id:
            return

        self.last_map = args
        rospy.loginfo("Map received. %d X %d." % (self.last_map.info.width, self.last_map.info.height))

        self.send_goal_to_segemantation(msg=self.last_map)

    def segmented_map_cb(self, args):
        pass

    def coverage_path_cb(self, args):
        pass


if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("roomba")
    node = RoomIPA()
    rospy.spin()
