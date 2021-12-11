#!/usr/bin/python3

import tf
import math
import rospy
import actionlib
from roomba.viz import Viz
from roomba.heap import Heap
from roomba.frontier import Frontier
from roomba.util import rotateQuaternion
from typing import *
from threading import Lock
from nav_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from map_msgs.msg import OccupancyGridUpdate
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
        self.prev_dist = math.inf  # Previous goal distance
        self.prev_time = None  # Last time we got feedback
        self.reached = False
        self.blacklist = []  # Banned frontiers b/c they're unreachable
        self.lock = Lock()  # For mutating the map structure(s)

        self.timeout = 15.0  # No goal progress timeout
        self.min_size = 0.75  # Minimum frontier size

        # Setup ROS hooks
        self.tf = tf.TransformListener()
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.pose_pub = rospy.Publisher('/estimatedpose', PoseStamped, queue_size=1)
        self.signal = rospy.Publisher('/roomba/map_ready', OccupancyGrid, queue_size=1)
        self.viz = Viz()

        # Setup actionlib & path service
        self.action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action.wait_for_server()  # Wait for move_base

        # Do a little spin to start
        self.do_spin()
        self.timer = rospy.Timer(rospy.Duration(1), lambda e: self.plan())

    def build_frontier(self, start: tuple, ref: Pose) -> Frontier:
        """Given a starting UKNW_CELL, build a frontier of connected cells."""
        # Initialize search
        queue = [start]
        pt = ref.position
        size = 1
        centre = Point(0, 0, 0)
        distance = math.inf
        points = []

        while queue:
            # Visit cell
            cell = queue.pop()
            wx, wy = self.map_to_world(*cell)

            # Update values
            points.append(Point(wx, wy, 0))
            size += 1
            centre.x += wx
            centre.y += wy
            distance = min(distance, math.sqrt((pt.x - wx) ** 2 + (pt.y - wy) ** 2))

            # Look at 8 neighbours
            for n in self.nhood_8(*cell):
                if self.is_frontier(*n):
                    self.visit[n] = True
                    queue.append(n)

        # Create frontier
        centre.x /= size
        centre.y /= size
        distance *= self.info.resolution
        size *= self.info.resolution
        return Frontier(ref, size, distance, centre, points)

    def get_cell(self, x: int, y: int) -> int:
        """Get cell occupancy value."""
        if x < 0 or y < 0 or x >= self.info.width or y >= self.info.height:
            raise IndexError(f'Index <{x}, {y}> is out of bounds')
        return self.data[x + y * self.info.width]

    def get_index(self, x: int, y: int) -> int:
        return x + y * self.info.width

    def index_to_map(self, i: int) -> tuple:
        return divmod(i, self.info.height)

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

        if self.visit.get((x, y), False):
            return False  # already visited

        for nx, ny in self.nhood_4(x, y):
            if self.get_cell(nx, ny) == self.FREE_CELL:
                return True  # has free neighbour
        return False

    def nearest_free(self, x: int, y: int) -> Optional[tuple]:
        """Find the nearest FREE_CELL to the given point."""
        queue = [(x, y)]
        visit = {(x, y): True}
        while queue:
            cell = queue.pop(0)

            if self.get_cell(*cell) == self.FREE_CELL:
                return cell
            for n in self.nhood_8(*cell):
                if not visit.get(n, False):
                    queue.append(n)
                    visit[n] = True
        return None

    def map_callback(self, msg: OccupancyGrid):
        """Update map and metadata."""

        # The callback may fire while we're in the middle of a search.
        # Use the lock to ensure data isn't mutilated during that time.
        with self.lock:
            self.info = msg.info
            self.data = msg.data

    def update_callback(self, msg: OccupancyGridUpdate):
        """Update map (with costmap update)."""
        with self.lock:
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
        self.pose_pub.publish(p)

        return p.pose

    def find_frontiers(self, pose: Pose) -> list:
        """Find a navigation goal given the current map."""

        # Lock map during search
        with self.lock:
            # Convert world coordinates to map indices
            pt = pose.position
            w_index = self.world_to_map(pt.x, pt.y)
            if not w_index:
                rospy.logwarn('Robot is out of world bounds')
                return []

            # Find nearest free cell to start search
            n_index = self.nearest_free(*w_index)
            if not n_index:
                rospy.logwarn('Could not find an empty cell nearby')
                n_index = w_index

            # DEBUG: Publish starting point
            # wx, wy = self.map_to_world(*n_index)
            # self.viz.mark_point(wx, wy, (1, 1, 0), 0.10)

            # Initialize search
            queue = [n_index]
            frontiers = []
            self.visit = {n_index: True}

            while queue:
                # Visit cell
                cell = queue.pop()

                # Look at 4 neighbours
                for n in self.nhood_4(*cell):
                    if self.visit.get(n, False):
                        continue  # already visited
                    self.visit[n] = True

                    occ = self.get_cell(*n)
                    if occ == self.FREE_CELL:
                        # Add to queue
                        queue.append(n)

                    if occ == self.UKNW_CELL:
                        # Build new frontier
                        f = self.build_frontier(n, pose)

                        if not self.is_allowed(f.centre):
                            continue  # Reject blacklisted frontiers
                        if f.size < self.min_size:
                            continue  # Reject small frontiers

                        frontiers.append(f)

        # Normalize and sort frontiers
        if len(frontiers) > 1:
            size = [f.size for f in frontiers]
            dist = [f.distance for f in frontiers]
            angl = [f.angle for f in frontiers]
            smin, smax = min(size), max(size)
            dmin, dmax = min(dist), max(dist)
            amin, amax = min(angl), max(angl)
            for f in frontiers:
                f.distance = (f.distance - dmin) / (dmax - dmin)
                f.size = (f.size - smin) / (smax - smin)
                f.angle = (f.angle - amin) / (amax - amin)
            frontiers.sort(key=lambda f: f.get_cost())
        return frontiers

    def goal_reached(self, status: GoalStatus, msg: MoveBaseResult):
        rospy.loginfo(f'Reached goal: <{self.prev_goal.x}, {self.prev_goal.y}>')
        if status == actionlib.GoalStatus.ABORTED:
            self.blacklist.append(self.prev_goal)
        self.reached = True

    def do_spin(self):
        """Do a (roughly) full spin."""
        rospy.loginfo('Spinning...')

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

    def send_goal(self, x: float, y: float, t: float):
        rospy.loginfo(f'New goal: <{x}, {y}>')
        goal = MoveBaseGoal()
        goal.target_pose.pose.position = Point(x, y, 0)
        goal.target_pose.pose.orientation = rotateQuaternion(Quaternion(0, 0, 0, 1), t)
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time()
        self.action.send_goal(goal, done_cb=self.goal_reached)

    def is_allowed(self, goal: Point) -> bool:
        for pt in self.blacklist:
            if abs(pt.x - goal.x) < 0.25 and abs(pt.y - goal.y) < 0.25:
                return False
        return True

    def plan(self):
        """Set a goal to move towards"""

        # Get current robot pose
        pose = self.get_pose()

        # Find frontiers
        frontiers = self.find_frontiers(pose)
        if not frontiers:
            rospy.loginfo('No frontiers found, are we done?')
            self.timer.shutdown()

            # Go back to charging station (origin)
            self.send_goal(0, 0, 0)

            # Signal next node
            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.header.stamp = rospy.Time()
            msg.info = self.info
            msg.data = self.data
            self.signal.publish(msg)
            return

        # DEBUG: Draw frontiers
        self.viz.clear_marks()
        self.viz.mark_frontiers(frontiers)

        # New goal or made progress
        f = frontiers[0]
        goal = f.centre

        same = self.prev_goal == goal
        self.prev_goal = goal
        if not same or self.prev_dist > f.distance:
            self.prev_time = rospy.get_time()
            self.prev_dist = f.distance

        # Timeout if no progress in a while
        delta_time = rospy.get_time() - self.prev_time
        if delta_time > self.timeout:
            self.blacklist.append(goal)
            rospy.loginfo(f'Blacklisted: <{goal.x}, {goal.y}>')
            # self.plan()
            return

        # Still pursuing previous goal
        if same:
            if self.reached and delta_time > 2:
                self.do_spin()
                self.reached = False
            return

        # Send goal to move_base
        dx = goal.x - pose.position.x
        dy = goal.y - pose.position.y
        angle = math.atan2(dy, dx)
        self.reached = False
        self.send_goal(goal.x, goal.y, angle)


if __name__ == '__main__':
    rospy.init_node('explore')
    node = Explorer()
    rospy.spin()
