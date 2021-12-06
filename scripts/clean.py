#!/usr/bin/python3

import tf
import math
import rospy
from typing import List
from roomba.grid import Grid
from move_base_msgs.msg import *
from sensor_msgs.msg import Image
from ipa_building_msgs.msg import *
from nav_msgs.msg import OccupancyGrid
from ortools.linear_solver import pywraplp
from actionlib import SimpleActionClient, GoalStatus
from roomba.util import getHeading, gen_sensor_img_with_data, grid_to_sensor_image
from geometry_msgs.msg import PoseStamped, Pose2D, Pose, Point32, Polygon


class Room:
    centre: Point32
    bounds: Polygon
    image: Image
    area: int
    path: List[PoseStamped]

    def contains(self, p: Pose2D) -> bool:
        """Check if room contains given pose."""
        min_pt = self.bounds.points[0]
        max_pt = self.bounds.points[1]
        return min_pt.x <= p.x <= max_pt.x and min_pt.y <= p.y <= max_pt.y


class Clean(object):
    def __init__(self):
        # Initialize variables
        self.battery = 100  # [0, 100]
        self.loss_rate = 0.01  # 1 level lost every 100 cells
        self.current_path = []  # Current path being followed

        # Get map
        rospy.loginfo('Waiting for map...')
        grid = rospy.wait_for_message('/map', OccupancyGrid)  # TODO: Change to '/roomba/map_ready' later
        self.grid = Grid(grid)
        image = grid_to_sensor_image(grid)

        # Get current pose
        rospy.loginfo('Waiting for pose transform...')
        listener = tf.TransformListener()
        listener.waitForTransform('base_link', 'map', rospy.Time(), rospy.Duration(10))

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = rospy.Time()
        self.charger_pose = listener.transformPose('map', pose)
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

        # Get map segmentation
        goal = MapSegmentationGoal()
        goal.input_map = image
        goal.map_resolution = self.grid.resolution
        goal.map_origin = self.grid.origin
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

        size = result.segmented_map.width * result.segmented_map.height
        for i in result.room_information_in_meter:
            r = Room()
            r.centre = i.room_center
            r.bounds = i.room_min_max
            data = [0 if x == i + 1 else 255 for x in result.segmented_map]
            r.image = gen_sensor_img_with_data(data, result.segmented_map)
            r.area = sum(data) - (255 * size)

            rospy.loginfo(f'Waiting for coverage in room #{i}...')
            centre = Pose2D(r.centre.x, r.centre.y, 0)
            r.path = self.get_coverage(r.image, result.map_resolution, result.map_origin, centre)

            # TODO: Maybe check if area too small like `mdp.py`
            self.rooms.append(r)

            # Check if charger (starting location) is this room
            if r.contains(pose):
                self.charger_room = i

    def get_coverage(self, image: Image, resolution: float, origin: Pose, pose: Pose2D) -> list:
        """Ask room_exploration_server for a path within given room image."""
        goal = RoomExplorationGoal()
        goal.input_map = image
        goal.map_resolution = resolution
        goal.map_origin = origin
        goal.robot_radius = 0.22  # Same as footprint
        goal.coverage_radius = 0.22  # TODO: This (and robot_radius) makes move_base behave awkward
        goal.starting_position = pose
        goal.planning_mode = 1  # Use the footprint, not FOV

        self.ipa_exp.send_goal_and_wait(goal)
        result = self.ipa_exp.get_result()
        path = result.coverage_path_pose_stamped

        # TODO: Simplify path

        return path

    def battery_lost(self, p1: PoseStamped, p2: PoseStamped) -> float:
        """Amount of battery lost traveling between poses."""
        p1, p2 = p1.pose.position, p2.pose.position
        d = math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
        n = d / self.grid.resolution
        return min(100, n * self.loss_rate)

    def path_cost(self, path: list) -> float:
        for i, cur in enumerate(path):
            if i + 1 < len(path):
                nxt = path[i + 1]
                # TODO

    def send_goal(self, p: PoseStamped):
        """Send navigation goal to move_base."""
        rospy.loginfo(f'Next goal: <{p.pose.position.x}, {p.pose.position.y}>')
        goal = MoveBaseGoal()
        goal.target_pose = p
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time()
        self.move_client.send_goal(goal, done_cb=self.follow_path)

    def follow_path(self, status: GoalStatus, _: MoveBaseActionResult):
        """Send the next pose in path as a goal."""
        if self.current_path:
            past = self.current_path.pop(0)
            now = self.current_path[0]
            self.battery -= self.battery_lost(past, now)
            self.send_goal(now)

    def start(self):
        """With rooms and paths all configured, start cleaning."""
        # TODO: Create adjacency graph
        N = len(self.rooms)
        graph = []
        for i in range(N):
            graph[i] = []
            for j in range(N):
                pass  # graph[i][j] = self.path_cost()

        # Repeat until all nodes visited
        visited = [False] * N
        while not all(visited):

            # (Re-)start with full battery and knowledge of previous visits
            sequence = self.solve_tsp(graph, visited, 100, 0, [])  # !!! THIS CAN BE SWAPPED-OUT !!! #

            # Update visited list
            for i in sequence:
                visited[i] = True

            # Convert room sequence to path:
            # 1. `sequence[0]` is always `self.charger_room`
            # 2. Return is *not* included in `sequence`
            path = []
            for i in sequence[1:]:  # Skip first room
                path += self.rooms[i].path
            path.append(self.charger_pose)  # Return to charger

            # Start following path
            self.current_path = path
            self.send_goal(path[0])

            # Wait until path complete before doing another round
            while not rospy.is_shutdown() and self.current_path:
                rospy.sleep(10)

    # !!! Path planning methods !!! #

    def solve_tsp(self, graph, visited, battery, this, path):
        """Solve the Traveling Salesman Problem (TSP) with added element of battery."""
        # graph : adjacency matrix, row-major
        # visited : boolean array
        # battery : integer [0, 100]
        # node : current node index

        # Terminal condition: out of battery
        back = graph[this][self.charger_room]  # Cost of going back to the charger
        if battery < back:
            return path

        # Set current node as visited
        visited = visited.copy()
        visited[this] = True

        # Add self to current path
        stump = path.copy()
        stump.append(this)
        path = stump

        # Terminal condition: all done
        if all(visited):
            return stump

        # Iterate over all neighbours
        neighbours = graph[this]
        for i, v in enumerate(neighbours):

            # Don't consider already visited (includes self)
            if visited[i]:
                continue

            # Extend path by visiting neighbour, using a bit of battery
            branch = self.solve_tsp(graph, visited, battery - v, i, stump)

            # Keep the longest path
            path = max(path, branch)

        return path

    #uses bin packing to deteermine order of rooms to clean.
    #pass a dictionary with room no as key and its area as
    #the value
    def bin_packing_planner(self, roomsandareas):
        data = {}
        data['weights'] = roomsandareas.values()
        data['items'] = list(range(len(roomsandareas.keys())))
        data['bins'] = data['items']
        data['bin_capacity'] = 150

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
            print()
            print('Number of bins used:', num_bins)
        else:
            print('The problem does not have an optimal solution.')


if __name__ == '__main__':
    rospy.init_node('clean')
    node = Clean()
    node.start()
    rospy.spin()
