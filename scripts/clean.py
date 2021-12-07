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
from actionlib import SimpleActionClient, GoalStatus
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
        self.loss_rate = 0.05  # 1 level lost every 20 cells
        self.current_path = []  # Current path being followed
        self.last_distance = 0  # Distance to current goal (every 10 seconds)
        self.last_goal = None  # Latest goal sent to move_base

        # Get map
        rospy.loginfo('Waiting for map...')
        grid = rospy.wait_for_message('/roomba/map_ready', OccupancyGrid)  # TODO: Change to '/roomba/map_ready' later
        self.grid = Grid(grid)
        image = grid_to_sensor_image(grid)

        # Get current pose
        rospy.loginfo('Waiting for initial pose...')
        self.charger_pose = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped).pose
        pose = self.charger_pose.pose
        pose = Pose2D(x=pose.position.x, y=pose.position.y, theta=getHeading(pose.orientation))

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
        goal.robot_radius = 0.22  # Same as footprint
        goal.room_segmentation_algorithm = 3

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
            if r.contains(pose):
                rospy.loginfo(f'Charger is in room #{i}')
                self.charger_room = i
                start = pose
            else:
                start = Pose2D(r.centre.x, r.centre.y, 0)

            # Get room coverage plan
            rospy.loginfo(f'Waiting for coverage in room #{i}...')
            r.path = self.get_coverage(r.image, result.map_resolution, result.map_origin, start)

            for p in r.path:  # Change '/map' to 'map'
                p.header.frame_id = 'map'
            r.cost = self.path_cost(r.path)

            self.rooms.append(r)

    def get_coverage(self, image: Image, resolution: float, origin: Pose, pose: Pose2D) -> list:
        """Ask room_exploration_server for a path within given room image."""
        goal = RoomExplorationGoal()
        goal.input_map = image
        goal.map_resolution = resolution
        goal.map_origin = origin
        goal.robot_radius = 0.22  # Same as footprint
        goal.coverage_radius = 0.44  # Double as footprint
        goal.starting_position = pose
        goal.planning_mode = 1  # Use the footprint, not FOV

        self.ipa_exp.send_goal_and_wait(goal)
        result = self.ipa_exp.get_result()
        path = result.coverage_path_pose_stamped

        # Smooth-out path
        smooth = [path[0]]
        dt = math.pi / 6
        for i in range(1, len(path) - 1):
            pt = getHeading(path[i - 1].pose.orientation)
            it = getHeading(path[i].pose.orientation)
            nt = getHeading(path[i + 1].pose.orientation)
            if abs(nt - it) >= dt or abs(it - pt) >= dt:
                smooth.append(path[i])
        smooth.append(path[-1])
        return smooth

    def get_pose(self) -> PoseStamped:
        """Get robot pose."""
        p = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose
        return p

    def get_distance(self, p1: PoseStamped, p2: PoseStamped) -> float:
        """Get Euclidean distance between two poses."""
        p1, p2 = p1.pose.position, p2.pose.position
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def battery_lost(self, p1: PoseStamped, p2: PoseStamped) -> float:
        """Amount of battery lost traveling between poses."""
        d = self.get_distance(p1, p2)
        n = d / self.grid.resolution
        return min(100, n * self.loss_rate)

    def path_cost(self, path: list) -> float:
        """Cost (battery loss) of following points on a path."""
        cost = 0
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            cost += self.battery_lost(p1, p2)
        return cost

    def send_goal(self, path=None):
        """Send navigation goal to move_base."""
        if path is not None:  # Assign new path
            if path:
                self.current_path = path
            else:
                rospy.loginfo('Empty path received, stopping...')

        # Reset progress
        p = self.current_path[0]
        self.last_goal = p
        self.last_distance = self.get_distance(self.get_pose(), p)

        # Send navigation goal
        rospy.loginfo(f'Next goal: <{p.pose.position.x}, {p.pose.position.y}>')
        goal = MoveBaseGoal()
        goal.target_pose = p
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time()
        self.move_client.send_goal(goal, done_cb=self.follow_path)

    def follow_path(self, status: GoalStatus, _: MoveBaseActionResult):
        """Send the next pose in path as a goal."""
        if self.current_path:
            # Update battery level
            past = self.current_path.pop(0)
            now = self.current_path[0]

            if now is None:
                rospy.loginfo('Moving onto next room...')
                self.current_path.pop(0)
            else:
                self.battery = max(0.0, self.battery - self.battery_lost(past, now))

            if self.battery > 0:  # Keep going
                rospy.loginfo(f'Battery level: {self.battery}')
                self.send_goal()
            else:  # Stop
                rospy.logerr('Out of battery!')
                self.send_goal([])

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
            # Convert bin to path
            path = []
            for i in b:
                path += self.rooms[i].path
                path.append(None)  # Signal to stop using battery between rooms
            path.append(self.charger_pose)  # Return to charger

            # Start following path
            self.send_goal(path)

            # Wait until path complete before doing another round
            while not rospy.is_shutdown() and self.current_path:
                rospy.sleep(10)
                p = self.current_path[0]
                d = self.get_distance(self.get_pose(), p)

                # Still pursuing last goal and have not made any
                # progress in the last 10 seconds -> give up
                if p == self.last_goal:
                    if d >= self.last_distance:
                        rospy.loginfo(f'No progress for 10 seconds, moving on...')
                        self.send_goal(self.current_path[1:])
                    else:
                        self.last_distance = d

        rospy.loginfo('All done cleaning!')

    # !!! Path planning methods !!! #

    #uses bin packing to deteermine order of rooms to clean.
    #pass a dictionary with room no as key and its area as
    #the value
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
