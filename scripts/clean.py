import math
import rospy
from roomba import util
from move_base_msgs.msg import *
from ipa_building_msgs.msg import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient, GoalStatus


class Clean(object):
    def __init__(self):
        # Initialize variables
        self.battery = 100  # [0, 100]

        # Get map
        grid = rospy.wait_for_message('/map', OccupancyGrid)  # TODO: Change to '/roomba/map_ready' later
        image = util.grid_to_sensor_image(grid)

        # Setup action client(s)
        client = SimpleActionClient('/room_exploration/room_exploration_server', RoomExplorationAction)
        client.wait_for_server()

        self.move_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        # Get coverage plan
        goal = RoomExplorationGoal()
        goal.input_map = image
        goal.map_resolution = grid.info.resolution
        goal.map_origin = grid.info.origin
        goal.robot_radius = 0.22  # Same as footprint
        goal.coverage_radius = 0.22
        goal.planning_mode = 1  # Use the footprint, not FOV
        client.send_goal_and_wait(goal)
        result = client.get_result()
        self.plan = result.coverage_path_pose_stamped
        self.origin = self.plan[0]
        self.returning = False
        self.index = 1

        # Start navigation
        self.send_goal(self.plan[1])

    def send_goal(self, p: PoseStamped):
        goal = MoveBaseActionGoal()
        goal.header.frame_id = 'map'
        goal.goal.target_pose = p
        self.move_client.send_goal(goal, done_cb=self.make_decision)

    @staticmethod
    def pose_distance(p1: PoseStamped, p2: PoseStamped) -> float:
        p1, p2 = p1.pose.position, p2.pose.position
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def make_decision(self, status: GoalStatus, _: MoveBaseActionResult):

        prv = self.plan[self.index - 1]
        cur = self.plan[self.index]
        nxt = self.plan[self.index + 1]

        if status == GoalStatus.ABORTED:
            rospy.logwarn(f'Goal to <{cur.pose.position.x}, {cur.pose.position.y}> was aborted!')
            self.send_goal(self.origin)  # When in doubt, return to origin

        # Reached origin to recharge, continue sweeping from last pose
        if self.returning:
            self.returning = False
            self.battery = 100  # Full recharge every time
            self.send_goal(self.plan[self.index])
            return

        # Reached goal, update battery expended
        to_current = self.pose_distance(prv, cur)
        to_next = self.pose_distance(cur, nxt)
        to_origin = self.pose_distance(cur, self.origin)
        self.battery -= to_current  # TODO: Tweak this

        # Would not be able to recover, return to recharge now
        if self.battery - to_next < to_origin:
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
