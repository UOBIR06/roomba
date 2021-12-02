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
        print('RoomIPA init.')

        # self._map_sub = rospy.Subscriber('/map', OccupancyGrid, None)
        # rospy.loginfo("Waiting for a map...")
        # try:
        #     ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        # except:
        #     rospy.logerr("Problem getting a map. Check that you have a map_server running: rosrun map_server map_server <mapname> ")
        #     sys.exit(1)
        # rospy.loginfo("Map received. %d X %d, %f px/m." % (ocuccupancy_map.info.width, ocuccupancy_map.info.height, ocuccupancy_map.info.resolution))

        self._sub_segmented_map = rospy.Subscriber('/room_segmentation/room_segmentation_server/segmented_map',
                                                   OccupancyGrid, self.segmented_map_cb)
        self._sub_coverage_path = rospy.Subscriber('/room_exploration/room_exploration_server/coverage_path', Path,
                                                   self.coverage_path_cb)

        # Get an action client
        self._sac_seg = actionlib.SimpleActionClient('/room_segmentation/room_segmentation_server',
                                                     MapSegmentationAction)
        self._sac_exp = actionlib.SimpleActionClient('/room_exploration/room_exploration_server',
                                                     RoomExplorationAction)
        rospy.loginfo('Waiting for action server to start.')

        self._sac_seg.wait_for_server()
        self._sac_exp.wait_for_server()
        rospy.loginfo('Action server started, sending goal.')

        # DynamicReconfigureClient
        self._drc_seg = dynamic_reconfigure.client.Client('/room_segmentation/room_segmentation_server')
        self._drc_seg.update_configuration({"room_segmentation_algorithm": 3})
        # self.send_goal_to_segemantation()  # for test

        self._drc_exp = dynamic_reconfigure.client.Client('/room_exploration/room_exploration_server')
        self._drc_exp.update_configuration({"room_exploration_algorithm": 8})
        # self.send_goal_to_exploration()  # for test

    def send_goal_to_segemantation(self, goal: MapSegmentationGoal = None, img_path: str = '') -> MapSegmentationResult:
        if not img_path:
            img_path = os.path.join(rospkg.RosPack().get_path('roomba'), 'data/sim_data/meeting.png')
        map_img: SensorImage = util.setup_image(img_path)

        # testing:
        if not goal:
            # Define the goal
            goal = MapSegmentationGoal()
            goal.input_map = map_img
            goal.map_origin.position.x = 0
            goal.map_origin.position.y = 0
            goal.map_resolution = 0.05
            goal.return_format_in_meter = False
            goal.return_format_in_pixel = True
            goal.robot_radius = 0.4

        rospy.loginfo("waiting for result")
        self._sac_seg.send_goal_and_wait(goal)
        response: MapSegmentationResult = self._sac_seg.get_result()
        rospy.loginfo("..........result................." + str(response.room_information_in_pixel))
        return response

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

    def segmented_map_cb(self, args):
        pass

    def coverage_path_cb(self, args):
        pass


if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("roomba")
    node = RoomIPA()
    rospy.spin()
