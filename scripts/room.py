#!/usr/bin/python3
"""
@author ir06
@date 23/11/2021
"""
import sys
import rospy
from tf.msg import tfMessage
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, PoseArray, Quaternion, Pose
from copy import deepcopy


class RoomExplorationIPA(object):
    def __init__(self):
        print('RoomExplorationIPA init.')

        self._charger_pose_received = False
        self._charger_pos = Pose()
        self._map = None

        self._map_sub = rospy.Subscriber('/map', OccupancyGrid, None)
        rospy.loginfo("Waiting for a map...")
        try:
            ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                       ocuccupancy_map.info.resolution))

        self._goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.publish_goal()

        self._initial_pose_subscriber = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self._initial_pose_callback)


    def _initial_pose_callback(self, pose):
        """ called when RViz sends a user supplied initial pose estimate """
        # self._particle_filter.set_initial_pose(pose)  # 🚩
        self._charger_pos = deepcopy(pose.pose.pose)
        self._charger_pose_received = True
        # self._cloud_publisher.publish(self._particle_filter.particlecloud)
        # initial path planning
        rospy.init_node()

    def publish_goal(self):
        goal = MoveBaseActionGoal()
        self._goal_pub.publish(goal)


    def talker():
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        rospy.init_node('Mover', anonymous=True)
        # rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            base_data = Twist()
            base_data.linear.x = 0.1
            pub.publish(base_data)
            # rate.sleep()

if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("room")
    node = RoomExplorationIPA()
    rospy.spin()
