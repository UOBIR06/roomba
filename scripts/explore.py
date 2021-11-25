#!/usr/bin/python3
import tf
import rospy
import actionlib
from typing import *
from threading import Lock

from nav_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from pf_localisation.util import *
from visualization_msgs.msg import Marker


class Explorer(object):
    FREE_CELL = 0
    FULL_CELL = 100
    UKNW_CELL = -1

    def __init__(self):
        """Create a new Explorer node. Effectively a singleton; don't instantiate multiple nodes."""

        # Init variables
        self.info = None  # Map metadata
        self.data = None  # Occupancy data
        self.visit = None  # Visited cell dictionary, for convenience
        self.prev_goal = None  # Last goal that was set
        self.lock = Lock()  # For mutating the map structure(s)

        # TODO: Keep playing with these values
        self.alpha = 0.7  # Distance to frontier
        self.beta = 0.3  # Size of frontier (decreases cost)
        self.gamma = 0.8  # Angle to frontier
        self.min_size = 20  # Minimum frontier size

        # Setup ROS hooks
        self.tf = tf.TransformListener()
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.mark_pub = rospy.Publisher('/marker', Marker, queue_size=100)
        self.pose_pub = rospy.Publisher('/estimatedpose', PoseStamped, queue_size=1)
        self.point_id = 0

        # Setup actionlib
        self.action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action.wait_for_server()  # Wait for move_base

        rospy.Timer(rospy.Duration(1), self.plan)

    def build_frontier(self, x: int, y: int) -> tuple:
        """Given a starting UKNW_CELL, build a frontier of connected cells."""
        # Initialize search
        queue = [(x, y)]
        cx = 0
        cy = 0
        n = 0

        while queue:
            # Visit cell
            x, y = queue.pop()
            self.visit[(x, y)] = 0  # Just use zero
            wx, wy = self.map_to_world(x, y)
            cx = cx + wx
            cy = cy + wy
            n = n + 1

            # Look at 8 neighbours
            for nx, ny in self.nhood_8(x, y):
                if self.is_frontier(nx, ny):
                    queue.append((nx, ny))

        # Calculate centre
        cx = cx / n
        cy = cy / n
        return n, cx, cy

    def get_cell(self, x: int, y: int) -> int:
        """Get cell occupancy value."""
        if x < 0 or y < 0 or x >= self.info.width or y >= self.info.height:
            raise IndexError(f'Index <{x}, {y}> is out of bounds')
        return self.data[x + y * self.info.width]

    def world_to_map(self, wx: int, wy: int) -> Optional[tuple]:
        """Convert world coordinates ('/map' frame) to cell indexes."""
        mx = int((wx - self.info.origin.position.x) / self.info.resolution + 0.5)
        my = int((wy - self.info.origin.position.y) / self.info.resolution + 0.5)
        if mx < 0 or my < 0 or mx >= self.info.width or my >= self.info.height:
            rospy.logerr(f'Map indices <{mx}, {my}> are out of bounds')
            return None
        return mx, my

    def map_to_world(self, mx: int, my: int) -> tuple:
        """Convert cell indexes to world coordinates."""
        wx = self.info.origin.position.x + (mx + 0.5) * self.info.resolution
        wy = self.info.origin.position.y + (my + 0.5) * self.info.resolution
        return wx, wy

    def nhood_4(self, x: int, y: int) -> list:
        """Make a list of in-bound 4-neighbours of a given point."""
        nhood = []
        if x > 0:
            nhood.append((x - 1, y))
        if x < self.info.width - 1:
            nhood.append((x + 1, y))
        if y > 0:
            nhood.append((x, y - 1))
        if y < self.info.height - 1:
            nhood.append((x, y + 1))
        return nhood

    def nhood_8(self, x: int, y: int) -> list:
        """Make a list of 8-neighbours of a given point."""
        nhood = self.nhood_4(x, y)
        if x > 0 and y > 0:
            nhood.append((x - 1, y - 1))
        if x > 0 and y < self.info.height - 1:
            nhood.append((x - 1, y + 1))
        if x < self.info.width - 1 and y > 0:
            nhood.append((x + 1, y - 1))
        if x < self.info.width - 1 and y < self.info.height - 1:
            nhood.append((x + 1, y + 1))
        return nhood

    def is_frontier(self, x: int, y: int) -> bool:
        """Test if given cell is a frontier."""
        if self.get_cell(x, y) != self.UKNW_CELL:
            return False

        if self.visit.get((x, y), -1) >= 0:
            return False  # already visited

        for nx, ny in self.nhood_4(x, y):
            if self.get_cell(nx, ny) == self.FREE_CELL:
                return True  # has free neighbour
        return False

    def nearest_free(self, x: int, y: int) -> Optional[tuple]:
        """Find the nearest FREE_CELL to the given point."""
        queue = [(x, y)]
        visit = {}
        while queue:
            cell = queue.pop()
            visit[cell] = True

            if self.get_cell(*cell) == self.FREE_CELL:
                return cell
            for neigh in self.nhood_8(*cell):
                if not visit.get(neigh, False):
                    queue.append(neigh)
        return None

    def clear_marks(self):
        """Clear RViz markers."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time()
        m.ns = 'point'
        m.action = 3  # Delete all objects in namespace
        self.mark_pub.publish(m)
        self.point_id = 0

    def mark_point(self, x: float, y: float, color=(1, 1, 1), scale=0.15):
        """Place a marker in RViz using '/marker' topic."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time()

        m.ns = 'point'
        m.id = self.point_id
        self.point_id += 1

        m.type = 1  # Use a cube
        m.action = 0  # Add/Modify mark

        m.pose.position = Point(x, y, 0)
        m.pose.orientation = Quaternion(0, 0, 0, 1)
        m.scale = Vector3(scale, scale, 0.01)

        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1

        m.lifetime = rospy.Duration(0)
        self.mark_pub.publish(m)

    def map_callback(self, msg: OccupancyGrid):
        """Update map and metadata."""

        # The callback may fire while we're in the middle of a search.
        # Use the lock to ensure data isn't mutilated during that time.
        with self.lock:
            self.info = msg.info
            self.data = msg.data

    def get_pose(self) -> Optional[Pose]:
        """Get estimated robot pose."""

        # Check if transform exists
        if not self.tf.canTransform('base_link', 'map', rospy.Time()):
            return

        # Get estimated robot pose
        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.header.stamp = rospy.Time()
        p = self.tf.transformPose('map', p)

        # DEBUG: Publish pose estimate
        self.clear_marks()
        self.pose_pub.publish(p)

        return p.pose

    def calculate_cost(self, n: int, x: float, y: float, t: float, i: int) -> float:
        dt = t - math.atan(x / y)
        return self.alpha * i - self.beta * n + self.gamma * dt

    def find_frontiers(self, pose) -> list:
        """Find a navigation goal given the current map."""

        # Lock map during search
        self.lock.acquire()

        # Convert world coordinates to map indices
        orientation = pose.orientation
        position = pose.position
        theta = getHeading(orientation)

        w_index = self.world_to_map(position.x, position.y)
        if not w_index:
            rospy.logwarn('Robot is out of world bounds')
            return []

        # Find nearest free cell to start search
        n_index = self.nearest_free(*w_index)
        if not n_index:
            rospy.logwarn('Could not find an empty cell nearby to start search')
            n_index = w_index

        # Initialize search
        queue = [(*n_index, 0)]
        frontiers = []
        self.visit = {}

        while queue:
            # Visit cell
            x, y, i = queue.pop()
            self.visit[(x, y)] = i

            # Look at 4 neighbours
            for nx, ny in self.nhood_4(x, y):
                if self.visit.get((nx, ny), -1) >= 0:
                    continue  # already visited

                occ = self.get_cell(nx, ny)
                if occ == self.FREE_CELL:
                    # Add to queue
                    queue.append((nx, ny, i + 1))

                if occ == self.UKNW_CELL:
                    # Build new frontier
                    n, cx, cy = self.build_frontier(nx, ny)

                    if n < self.min_size:
                        continue  # Reject small frontiers
                    # TODO: Reject unreachable frontiers

                    cost = self.calculate_cost(n, cx, cy, theta, i)
                    frontiers.append((cx, cy, cost))

        # No longer need map, release lock
        self.lock.release()

        # Sort and return frontiers
        frontiers.sort(key=lambda f: f[2])  # Use cost as key
        return frontiers

    def feedback_callback(self, msg: MoveBaseFeedback):
        rospy.loginfo(f'Feedback: {msg}')

    def done_callback(self, status: GoalStatus, msg: MoveBaseResult):
        rospy.loginfo(f'Reached goal: {msg}')

    def do_spin(self):
        """Do a (roughly) full spin."""
        hz = 10
        step = 1.0  # rad/s
        full_time = (2 * math.pi) / step
        end_count = full_time * hz  # count
        count = 0

        cmd = Twist()
        cmd.angular.z = step
        rate = rospy.Rate(hz)

        while not rospy.is_shutdown() and count < end_count:
            self.cmd_pub.publish(cmd)
            rate.sleep()
            count += 1

    def plan(self, event):
        """Set a goal to move towards"""

        # Get current robot pose
        pose = None
        while not pose:
            self.do_spin()  # Spin prompts SLAM to publish data
            pose = self.get_pose()

        # DEBUG: Publish starting point
        self.clear_marks()
        self.mark_point(pose.position.x, pose.position.y, (1, 1, 0))

        # Find frontiers
        frontiers = self.find_frontiers(pose)
        if not frontiers:
            # TODO: Go back to charging station (origin)
            # TODO: Notify next node to start (sweeping)
            sys.exit(0)

        # DEBUG: Publish goal
        x, y, cost = frontiers[0]
        self.mark_point(x, y, (0, 1, 0))

        # Send goal to move_base
        goal = MoveBaseGoal()
        goal.target_pose.pose.position = Point(x, y, 0)
        goal.target_pose.pose.orientation.w = 1  # TODO: Use facing direction
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time()
        self.action.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)


if __name__ == '__main__':
    rospy.init_node('explore')
    node = Explorer()
    rospy.spin()
