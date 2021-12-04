#!/usr/bin/python3
import tf
import math
import rospy
from move_base_msgs.msg import *
from sensor_msgs.msg import Image
from roomba.util import getHeading
from ipa_building_msgs.msg import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose2D
from actionlib import SimpleActionClient, GoalStatus


class Clean(object):
    def __init__(self):
        # Initialize variables
        self.battery = 100  # [0, 100]
        self.loss_rate = 0.01
        self.returning = False
        self.index = 1

        # Get map
        rospy.loginfo('Waiting for map...')
        grid = rospy.wait_for_message('/map', OccupancyGrid)  # TODO: Change to '/roomba/map_ready' later
        self.resolution = grid.info.resolution
        image = self.grid_to_image(grid)

        # Get current pose
        listener = tf.TransformListener()
        listener.waitForTransform('base_link', 'map', rospy.Time(), rospy.Duration(10))
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = rospy.Time()
        pose = listener.transformPose('map', pose).pose
        pose = Pose2D(x=pose.position.x, y=pose.position.y, theta=getHeading(pose.orientation))

        # Setup action client(s)
        rospy.loginfo('Waiting for ipa_room_exploration...')
        client = SimpleActionClient('/room_exploration/room_exploration_server', RoomExplorationAction)
        client.wait_for_server()

        rospy.loginfo('Waiting for move_base...')
        self.move_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        # Get coverage plan
        rospy.loginfo('Waiting for coverage plan...')
        goal = RoomExplorationGoal()
        goal.input_map = image
        goal.map_resolution = grid.info.resolution
        goal.map_origin = grid.info.origin
        goal.robot_radius = 0.22  # Same as footprint
        goal.coverage_radius = 0.22  # TODO: This (and robot_radius) makes move_base behave awkward
        goal.starting_position = pose
        goal.planning_mode = 1  # Use the footprint, not FOV
        client.send_goal_and_wait(goal)
        result = client.get_result()
        self.plan = result.coverage_path_pose_stamped
        self.origin = self.plan[0]

        # TODO: Smooth-out path

        # Start navigation
        self.send_goal(self.plan[1])

    def send_goal(self, p: PoseStamped):
        rospy.loginfo(f'Next goal: <{p.pose.position.x}, {p.pose.position.y}>')
        goal = MoveBaseGoal()
        goal.target_pose = p
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time()
        self.move_client.send_goal(goal, done_cb=self.make_decision)

    def battery_lost(self, p1: PoseStamped, p2: PoseStamped) -> float:
        p1, p2 = p1.pose.position, p2.pose.position
        d = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
        n = d / self.resolution
        return min(100, n * self.loss_rate)

    @staticmethod
    def grid_to_image(grid: OccupancyGrid) -> Image:
        data = bytes(map(lambda x: 255 if x == 0 else 0, grid.data))
        image = Image()
        image.width = grid.info.width
        image.height = grid.info.height
        image.encoding = 'mono8'
        image.is_bigendian = int(sys.byteorder == 'big')
        image.step = image.width
        image.data = data
        return image

    def make_decision(self, status: GoalStatus, _: MoveBaseActionResult):
        prv = self.plan[self.index - 1]
        cur = self.plan[self.index]
        nxt = self.plan[self.index + 1]

        if status == GoalStatus.ABORTED:
            rospy.logwarn(f'Goal to <{cur.pose.position.x}, {cur.pose.position.y}> was aborted!')
            self.send_goal(self.origin)  # When in doubt, return to origin

        # Reached origin to recharge, continue sweeping from last pose
        if self.returning:
            rospy.loginfo('Recharged, continue sweeping.')
            self.returning = False
            self.battery = 100  # Full recharge every time
            self.send_goal(self.plan[self.index])
            return

        # Reached goal, update battery expended
        to_current = self.battery_lost(prv, cur)
        to_next = self.battery_lost(cur, nxt)
        to_origin = self.battery_lost(cur, self.origin)
        self.battery -= to_current  # TODO: Tweak this

        # Would not be able to recover, return to recharge now
        if self.battery - to_next < to_origin:
            rospy.loginfo('Battery too low, returning.')
            self.returning = True
            self.send_goal(self.origin)
            return

        # Can continue along path
        self.index += 1
        self.send_goal(nxt)


if __name__ == '__main__':
    rospy.init_node('clean')
    node = Clean()
    rospy.spin()
