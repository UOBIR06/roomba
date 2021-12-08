#!/usr/bin/python3
import cv2

import tf
import math
import rospy
import numpy as np
from roomba.grid import Grid
from nav_msgs.srv import GetPlan
from move_base_msgs.msg import *
from sensor_msgs.msg import Image
from typing import List, Optional
from ipa_building_msgs.msg import *
from nav_msgs.msg import OccupancyGrid
from ortools.linear_solver import pywraplp
from actionlib import SimpleActionClient, GoalStatus, GoalID
from roomba.util import getHeading, gen_sensor_img_with_data, grid_to_sensor_image
from geometry_msgs.msg import PoseStamped, Pose2D, Pose, Point, Point32, Polygon, Quaternion, PoseWithCovarianceStamped


class Room:
    centre: Point32
    bounds: Polygon
    image: Image
    area: int
    path: List[PoseStamped]
    cost: float

    def contains(self, p: Pose2D) -> bool:
        """Check if room contains given pose."""
        min_pt = self.bounds.points[0]
        max_pt = self.bounds.points[1]
        return min_pt.x <= p.x <= max_pt.x and min_pt.y <= p.y <= max_pt.y


class Clean(object):
    def __init__(self):
        # Initialize variables
        self.battery = 100.0  # [0, 100]
        self.loss_rate = 0.03  # 1 level lost every 33 cells
        self.current_bin = []  # Current sequence of rooms
        self.current_path = []  # Current path being followed
        self.bin_index = 0  # Index within bin
        self.path_index = 0  # Index within path

        self.at_charger = True  # Flag to indicate end of bin
        self.returning = False  # Flag to indicate heading to charger

        self.last_distance = 0  # Distance to current goal (every 10 seconds)
        self.last_goal = None  # Latest goal sent to move_base
        self.last_time = None  # Last time progress was made
        self.timeout = 10  # How long to wait before giving-up on a point

        # Get map
        rospy.loginfo('Waiting for map...')
        grid = rospy.wait_for_message('/roomba/map_ready', OccupancyGrid)
        self.grid = Grid(grid)  # Hmmm. Do you just want to use the data structure?
        image = grid_to_sensor_image(grid)

        # Get current pose
        # [Not important]: explorer should be reading this and save as charging point.
        # AND when explorer finished, it may not be in charging point and still have enough battery left.
        rospy.loginfo('Waiting for initial pose...')
        init = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        stamped = PoseStamped()
        stamped.header = init.header
        stamped.pose = init.pose.pose
        self.charger_pose = stamped
        pose = Pose2D(x=stamped.pose.position.x, y=stamped.pose.position.y, theta=getHeading(stamped.pose.orientation))

        # Setup action client(s)
        rospy.loginfo('Waiting for ipa_*_server(s)...')
        self.ipa_exp = SimpleActionClient('/room_exploration/room_exploration_server', RoomExplorationAction)
        self.ipa_exp.wait_for_server()
        self.ipa_seg = SimpleActionClient('/room_segmentation/room_segmentation_server', MapSegmentationAction)
        self.ipa_seg.wait_for_server()

        rospy.loginfo('Waiting for move_base...')
        self.move_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()
        self.move_service = rospy.ServiceProxy('/nav/make_plan', GetPlan, True)

        # Get map segmentation
        goal = MapSegmentationGoal()
        goal.input_map = image
        goal.map_resolution = self.grid.resolution
        goal.map_origin = self.grid.info.origin
        goal.return_format_in_pixel = True
        goal.return_format_in_meter = True
        goal.robot_radius = 0.31  # Same as footprint; Doesn't really matter; Should be, but sth's wrong with server.
        goal.room_segmentation_algorithm = 1  # morphological segmentation

        rospy.loginfo('Waiting for segmentation...')
        self.ipa_seg.send_goal_and_wait(goal)
        result = self.ipa_seg.get_result()

        # Create rooms and their paths
        self.rooms = []
        self.charger_room = -1

        image = np.frombuffer(result.segmented_map.data, dtype=np.intc)
        for i, info in enumerate(result.room_information_in_meter):
            r = Room()
            r.centre = info.room_center
            r.bounds = info.room_min_max
            data = [255 if x == i + 1 else 0 for x in image]
            r.image = gen_sensor_img_with_data(data, result.segmented_map)
            r.area = np.count_nonzero(data) * result.map_resolution ** 2  # m^2

            # Check if charger (starting location) is this room
            # FIXME because contains just checkes the bounding box;
            # so it's possible other room's overlapping
            # fortunately, charger_room is only a flag;
            # unfortunately, start is not. But it dosen't affect too much.
            # and according to ipa's code, I believe we should leave it this way: pass it to all possible rooms
            if r.contains(pose):  # and self.charger_room != -1
                rospy.loginfo(f'Charger is in room #{i}')
                self.charger_room = i
                start = pose
            else:
                # actually it should be the nearest point from last position; but we can't get that; NOT for now
                start = Pose2D(r.centre.x, r.centre.y, 0)

            # Get room coverage plan
            rospy.loginfo(f'Waiting for coverage in room #{i}...')
            r.path = self.get_coverage(r.image, result.map_resolution, result.map_origin, start)
            r.cost = self.path_cost(r.path)

            # room's too small; no path found
            if r.cost:
                self.rooms.append(r)

    def get_coverage(self, image: Image, resolution: float, origin: Pose, pose: Pose2D) -> Optional[list]:
        """Ask room_exploration_server for a path within given room image."""
        goal = RoomExplorationGoal()
        goal.input_map = image
        goal.map_resolution = resolution
        goal.map_origin = origin
        goal.robot_radius = 0.31  # Same as footprint is 0.22
        goal.coverage_radius = 0.31  # Double as footprint is 0.22
        goal.starting_position = pose
        goal.planning_mode = 1  # Use the footprint, not FOV

        self.ipa_exp.send_goal_and_wait(goal)
        result = self.ipa_exp.get_result()

        if not result:
            return
        path = result.coverage_path_pose_stamped

        # room's too small; no path found
        if not len(path):
            return

        # Smooth-out path
        smooth = [path[0]]
        dt = math.pi / 6
        for i in range(1, len(path) - 1):
            pt = getHeading(path[i - 1].pose.orientation)
            it = getHeading(path[i].pose.orientation)
            nt = getHeading(path[i + 1].pose.orientation)
            if abs(nt - it) >= dt or abs(it - pt) >= dt:
                smooth.append(path[i])  # [ASK] what's this do? simply duplication works?
        smooth.append(path[-1])
        return smooth

    #
    # up: initialisation
    # down: start process
    #

    def get_pose(self) -> PoseStamped:
        """Get robot pose."""
        p = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        stamped = PoseStamped()
        stamped.header = p.header
        stamped.pose = p.pose.pose
        return stamped

    def get_distance(self, p1: PoseStamped, p2: PoseStamped) -> float:
        """Get Euclidean distance between two poses."""
        p1, p2 = p1.pose.position, p2.pose.position
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def battery_lost(self, p1: PoseStamped, p2: PoseStamped) -> float:
        """Amount of battery lost traveling between poses."""
        d = self.get_distance(p1, p2)
        n = d / self.grid.resolution
        return min(100, n * self.loss_rate)

    def path_cost(self, path: list) -> float:
        """Cost (battery loss) of following points on a path."""
        cost = 0
        if path and len(path):
            for i in range(len(path) - 1):
                p1 = path[i]
                p2 = path[i + 1]
                cost += self.battery_lost(p1, p2)
        return cost

    def send_goal(self, p: PoseStamped):
        """Send navigation goal to move_base."""
        rospy.loginfo(f'Next goal: <{p.pose.position.x}, {p.pose.position.y}>')

        goal = MoveBaseGoal()
        goal.target_pose = p
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time()
        self.move_client.send_goal(goal, done_cb=self.follow_path)

    def get_next_goal(self) -> Optional[PoseStamped]:
        """Get the next goal in path, or bin, or nothing."""
        # Update indices
        self.path_index += 1
        if self.path_index >= len(self.current_path):
            self.path_index = 0

            # Done with current bin, no more goals
            self.bin_index += 1
            if self.bin_index >= len(self.current_bin):
                return None

            # More rooms to clean
            r = self.current_bin[self.bin_index]
            self.current_path = self.rooms[r].path

        # Return next goal
        return self.current_path[self.path_index]

    def send_next_goal(self):
        """Send the next goal in line."""
        goal = self.get_next_goal()
        if goal is None:
            rospy.loginfo('Returning to recharge')
            self.returning = True
            self.send_goal(self.charger_pose)
        else:
            self.send_goal(goal)

    def follow_path(self, status: GoalStatus, _: MoveBaseActionResult):
        """Send the next pose in path as a goal."""

        # If we were returning, now we're done
        if self.returning:
            self.at_charger = True
            return

        if self.path_index > 0:  # Update battery level
            past = self.current_path[self.path_index - 1]
            now = self.current_path[self.path_index]
            self.battery = max(0.0, self.battery - self.battery_lost(now, past))
            rospy.loginfo(f'Battery: {self.battery}')

            if self.battery == 0:
                rospy.logerr('Out of battery!')
                # [Future] we can send the robot go home now
                # and we'll have all the bin_index and path_index in positino when charging done
                return

        self.send_next_goal()

    def start(self):
        """With rooms and paths all configured, start cleaning."""

        # Sanity checks
        if not self.rooms:
            rospy.logerr('No rooms found, aborting!')
            return
        if self.charger_room < 0:
            rospy.logerr('Charger room not found, aborting!')
            return

        # Get bins (i.e. trips)
        rooms = {}
        for i, r in enumerate(self.rooms):
            rooms[i] = r.cost
        bins = self.bin_packing_planner(rooms)

        # Visit all the bins
        for b in bins:
            # Set current bin
            self.current_bin = b
            self.bin_index = 0

            # Set current path
            r = self.current_bin[self.bin_index]
            self.current_path = self.rooms[r].path
            self.path_index = 0

            # Start cleaning
            self.battery = 100  # Start on full charge
            self.at_charger = False
            self.returning = False
            self.send_goal(self.current_path[0])

            # Wait until round complete before going again
            while not rospy.is_shutdown() and not self.at_charger:
                rospy.sleep(2)

                if self.battery == 0:
                    rospy.logerr('Out of battery! HELPPP')
                    return

                if not self.returning:
                    goal = self.current_path[self.path_index]
                    same_goal = self.last_goal == goal
                    self.last_goal = goal

                    pose = self.get_pose()
                    distance = self.get_distance(pose, goal)

                    # [ASK]why is self.last_distance > distance when same_goal? robot moved?
                    if not same_goal or self.last_distance > distance:
                        self.last_distance = distance
                        self.last_time = rospy.get_time()

                    if rospy.get_time() - self.last_time > self.timeout:
                        rospy.logwarn(f'No progress for {self.timeout} seconds, moving on...')
                        self.send_next_goal()

        rospy.loginfo('All done cleaning!')

    # Uses bin packing to determine order of rooms to clean.
    # Pass a dictionary with room no. as key and its area as
    # the value
    def bin_packing_planner(self, roomsandareas):
        data = {}
        data['weights'] = list(roomsandareas.values())
        data['items'] = list(range(len(roomsandareas.keys())))
        data['bins'] = data['items']
        data['bin_capacity'] = 100

        solver = pywraplp.Solver.CreateSolver('SCIP')
        # Variables
        # x[i, j] = 1 if item i is packed in bin j.
        x = {}
        for i in data['items']:
            for j in data['bins']:
                x[(i, j)] = solver.IntVar(0, 1, 'x_%i_%i' % (i, j))

        # y[j] = 1 if bin j is used.
        y = {}
        for j in data['bins']:
            y[j] = solver.IntVar(0, 1, 'y[%i]' % j)

        # Constraints
        # Each item must be in exactly one bin.
        for i in data['items']:
            solver.Add(sum(x[i, j] for j in data['bins']) == 1)

        # The amount packed in each bin cannot exceed its capacity.
        for j in data['bins']:
            solver.Add(
                sum(x[(i, j)] * data['weights'][i] for i in data['items']) <= y[j] *
                data['bin_capacity'])

        # Objective: minimize the number of bins used.
        solver.Minimize(solver.Sum([y[j] for j in data['bins']]))

        status = solver.Solve()

        if status == pywraplp.Solver.OPTIMAL:
            bins = []
            num_bins = 0.
            for j in data['bins']:
                if y[j].solution_value() == 1:
                    bin_items = []
                    bin_weight = 0
                    for i in data['items']:
                        if x[i, j].solution_value() > 0:
                            bin_items.append(i)
                            bin_weight += data['weights'][i]
                    if bin_weight > 0:
                        num_bins += 1
                        print('Bin number', j)
                        print('  Items packed:', bin_items)
                        print('  Total weight:', bin_weight)
                        print()
                        bins.append(bin_items)
            print()
            print('Number of bins used:', num_bins)
            return bins
        else:
            rospy.logerr('The problem does not have an optimal solution.')
            return []


if __name__ == '__main__':
    rospy.init_node('clean')
    node = Clean()
    node.start()
    rospy.spin()
