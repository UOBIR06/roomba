#!/usr/bin/python3

import tf
import math
import rospy
import actionlib
from roomba import *
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

        self.timeout = 10   # No goal progress timeout
        self.min_dist = 5   # Goal reached tolerance
        self.min_size = 20  # Minimum frontier size

        # Setup ROS hooks
        self.tf = tf.TransformListener()
        #self.map_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.map_callback)
        #self.update_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.update_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.mark_pub = rospy.Publisher('/marker', Marker, queue_size=1000)
        self.pose_pub = rospy.Publisher('/estimatedpose', PoseStamped, queue_size=1)
        self.signal = rospy.Publisher('/roomba/map_ready', OccupancyGrid, queue_size=1)
        #self.pathsrv = rospy.ServiceProxy('move_base', nav_msgs.srv.GetPlan, True)
        self.point_id = 0

        # Setup actionlib & path service
        self.action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action.wait_for_server()  # Wait for move_base

        # Do a little spin to start
        self.do_spin()
        self.timer = rospy.Timer(rospy.Duration(1), lambda e: self.plan())

    def find_path(self, start: tuple, goal: tuple) -> Optional[list]:
        """Find a path between two points on the map."""

        # Heuristic function
        def h(pt: tuple) -> float:
            dx = pt[0] - start[0]
            dy = pt[1] - start[1]
            return math.sqrt(dx**2 + dy**2)

        def reconstruct(m: dict, s: tuple) -> list:
            path = []
            while s in camefrom:
                # DEBUG: Show path
                wx, wy = self.map_to_world(*s)
                self.mark_point(wx, wy, (1, 0, 0), 0.05)

                s = camefrom[s]
                path.append(s)
            return path

        heap = Heap()
        heap.push(start, h(start))
        gscore = {start: 0}
        camefrom = {}

        while heap:
            current = heap.pop()
            if current == goal:
                return reconstruct(camefrom, current)

            score = gscore.get(current) + 1  # All neighbours are just one cell away
            for n in self.nhood_8(*current):
                if self.get_cell(*n) > 0:
                    continue  # Ignore occupied cells

                if score < gscore.get(n, math.inf):
                    camefrom[n] = current
                    gscore[n] = score
                    if not heap.has(n):
                        heap.push(n, score + h(n))

        return None

    def build_frontier(self, x: int, y: int, ref: Pose) -> Frontier:
        """Given a starting UKNW_CELL, build a frontier of connected cells."""
        # Initialize search
        queue = [(x, y)]
        pt = ref.position
        f = Frontier()
        f.size = 1
        f.reference = ref

        while queue:
            # Visit cell
            x, y = queue.pop()
            self.visit[(x, y)] = 0  # Just use zero
            wx, wy = self.map_to_world(x, y)

            cell = Point(wx, wy, 0)
            f.points.append(cell)
            f.size += 1
            f.centre.x += wx
            f.centre.y += wy

            dist = math.sqrt((pt.x - wx)**2 + (pt.y - wy)**2)
            if dist < f.distance:
                f.distance = dist

            # Look at 8 neighbours
            for nx, ny in self.nhood_8(x, y):
                if self.is_frontier(nx, ny):
                    queue.append((nx, ny))

        f.centre.x /= f.size
        f.centre.y /= f.size
        return f

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
            wx, wy = self.map_to_world(*n_index)
            self.mark_point(wx, wy, (1, 1, 0), 0.10)

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
                        f = self.build_frontier(nx, ny, pose)
                        if f.size < self.min_size:
                            continue  # Reject small frontiers

                        # try:
                        #     start = PoseStamped()
                        #     start.header.frame_id = 'map'
                        #     start.pose = pose
                        #
                        #     end = PoseStamped()
                        #     end.header.frame_id = 'map'
                        #     end.pose = Pose(f.centre, Quaternion(0, 0, 0, 1))
                        #
                        #     resp = self.pathsrv(start, end, 0)
                        # except Exception as e:
                        #     rospy.logwarn(f'Path service returned error: {e}')
                        #     continue
                        #
                        # if not resp.plan.poses:
                        #     continue  # No path found
                        # f.path = resp.plan

                        # start = self.world_to_map(pose.position.x, pose.position.y)
                        # goal = self.world_to_map(f.centre.x, f.centre.y)
                        # f.path = self.find_path(start, goal)
                        # if not f.path:
                        #    continue

                        frontiers.append(f)

        # Sort and filter frontiers
        frontiers.sort(key=lambda k: k.get_cost())
        return list(filter(lambda k: self.is_allowed(k.centre), frontiers))

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
            dx = abs(pt.x - goal.x)
            dy = abs(pt.y - goal.y)
            if dx < 0.25 and dy < 0.25:
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

            # Quit
            sys.exit(0)

        # DEBUG: Draw frontiers
        self.clear_marks()
        for f in frontiers:
            for pt in f.points:
                self.mark_point(pt.x, pt.y, (0, 0, 1), 0.05)
            self.mark_point(f.centre.x, f.centre.y, (0, 1, 0), 0.10)

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
            self.plan()
            return

        # Still pursuing previous goal
        if same:
            if self.reached and delta_time > self.timeout / 2:
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
