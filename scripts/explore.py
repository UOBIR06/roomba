#!/usr/bin/python3
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import MapMetaData, OccupancyGrid
import tf
from tf.msg import tfMessage
import math
from visualization_msgs.msg import Marker
from pf_localisation.util import *

class Explorer(object):
    FREE_CELL = 0
    FULL_CELL = 100
    UKNW_CELL = -1

    def __init__(self):
        self.tf = tf.TransformListener()
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.pose_pub = rospy.Publisher('/estimatedpose', PoseStamped, queue_size=1)
        self.mark_pub = rospy.Publisher('/marker', Marker, queue_size=100)
        self.point_id = 0

        # TODO: Keep playing with these values
        self.alpha = 0.7    # Distance to frontier
        self.beta = 0.3     # Size of frontier (decreases cost)
        self.gamma = 0.8    # Angle to frontier
        self.min_size = 20  # Minimum frontier size

    def build_frontier(self, x, y):
        # Initialize search
        queue = [ (x, y) ]
        cx = 0
        cy = 0
        n = 0

        while queue:
            # Visit cell
            x, y = queue.pop()
            self.visit[(x, y)] = 0 # Just use zero
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

    def get_cell(self, x, y):
        if x < 0 or y < 0 or x >= self.info.width or y >= self.info.height:
            raise IndexError(f'Index <{x}, {y}> is out of bounds')
        return self.data[x + y * self.info.width]

    def world_to_map(self, wx, wy):
        mx = int((wx - self.info.origin.position.x) / self.info.resolution + 0.5)
        my = int((wy - self.info.origin.position.y) / self.info.resolution + 0.5)
        if mx < 0 or my < 0 or mx >= self.info.width or my >= self.info.height:
            rospy.logerr(f'Map indices <{mx}, {my}> are out of bounds')
            return None
        return mx, my

    def map_to_world(self, mx, my):
        wx = self.info.origin.position.x + (mx + 0.5) * self.info.resolution
        wy = self.info.origin.position.y + (my + 0.5) * self.info.resolution
        return wx, wy

    def nhood_4(self, x, y):
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

    def nhood_8(self, x, y):
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

    def is_frontier(self, x, y):
        if self.get_cell(x, y) != self.UKNW_CELL:
            return False

        if self.visit.get((x, y), -1) >= 0:
            return False  # already visited

        for nx, ny in self.nhood_4(x, y):
            if self.get_cell(nx, ny) == self.FREE_CELL:
                return True # has free neighbour
        return False

    def nearest_free(self, x, y):
        queue = [ (x, y) ]
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
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time()
        m.ns = 'point'
        m.action = 3 # Delete all objects in namespace
        self.mark_pub.publish(m)
        self.point_id = 0

    def mark_point(self, x, y, color = [1, 1, 1], scale = 0.15):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time()

        m.ns = 'point'
        m.id = self.point_id
        self.point_id += 1

        m.type = 1 # Use a cube
        m.action = 0 # Add/Modify mark

        m.pose.position = Point(x, y, 0)
        m.pose.orientation = Quaternion(0, 0, 0, 1)
        m.scale = Vector3(scale, scale, 0.01)

        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1

        m.lifetime = rospy.Duration(0)
        self.mark_pub.publish(m)

    def map_callback(self, msg):
        self.head = msg.header
        self.info = msg.info
        self.data = msg.data

        # Check if transform exists
        if not self.tf.canTransform('base_link', 'map', rospy.Time()):
            return

        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.header.stamp = rospy.Time()
        p = self.tf.transformPose('map', p)
        
        # DEBUG: Publish pose estimate
        self.clear_marks()
        self.pose_pub.publish(p)

        # Convert world coordinates to map indices
        theta = getHeading(p.pose.orientation)
        p = p.pose.position
        p = self.world_to_map(p.x, p.y)
        if not p:
            return
        
        # Find nearest free cell to start search
        p = self.nearest_free(*p)
        if not p:
            return
        px, py = p

        # DEBUG: Publish starting point
        wx, wy = self.map_to_world(px, py)
        self.mark_point(wx, wy, [1, 1, 0])
        
        # Initialize search
        queue = [ (px, py, 0) ]
        frontiers = []
        self.visit = {}

        while queue:
            # Visit cell
            x, y, l = queue.pop()
            self.visit[(x, y)] = l

            # Look at 4 neighbours
            for nx, ny in self.nhood_4(x, y):
                if self.visit.get((nx, ny), -1) >= 0:
                    continue # already visited

                occ = self.get_cell(nx, ny)
                if occ == self.FREE_CELL:
                    queue.append((nx, ny, l + 1)) # add to queue
                if occ == self.UKNW_CELL:
                    # Current cell is guaranteed to be FREE_CELL and so
                    # if it has an UKNW_CELL neighbour that's a frontier
                    n, cx, cy = self.build_frontier(nx, ny)
                    if n < self.min_size:
                        continue
                    # TODO: Ignore frontiers that are unreachable using move_base services
                    frontiers.append((n, cx, cy, l)) # num. of cells, centre, path length (nearest cell)

        # Check for frontiers
        if not frontiers:
            rospy.loginfo('No frontiers found. Has the map been fully explored?')
            return

        # Find min & max for normalization
        # NOTE: Commented are the advice of Gao et al. which is not very good
        # because it causes the frontier with the minimum value to be normalized
        # as 0, and the maximum value as 1. This screws with the cost calculations
        # as seen from experience. Instead, dividing by the total is better.

        #min_len = math.inf
        #max_len = -1
        #min_num = math.inf
        #max_num = -1
        #min_ang = math.inf
        #max_ang = -1
        t_len = 0
        t_num = 0
        t_ang = 0
        for n, x, y, l in frontiers:
            #min_len = min(min_len, l)
            #max_len = max(max_len, l)
            #min_num = min(min_num, n)
            #max_num = max(max_num, n)

            t = abs(theta - math.atan(y / x))
            t_len += l
            t_num += n
            t_ang += t
            #min_ang = min(min_ang, t)
            #max_ang = max(max_ang, t)

        #len_range = max_len - min_len
        #num_range = max_num - min_num
        #ang_range = max_ang - min_ang 

        # Calculate costs
        min_cost = math.inf
        goal = None
        for n, x, y, l in frontiers:
            t = abs(theta - math.atan(y / x))

            # Normalize
            #l = (l - min_len) / len_range
            #n = (n - min_num) / num_range
            #t = (t - min_ang) / ang_range
            l = l / t_len
            n = n / t_num
            t = t / t_ang

            assert 0 <= l <= 1
            assert 0 <= n <= 1
            assert 0 <= t <= 1

            # Update cost & goal
            cost = self.alpha * l - self.beta * n + self.gamma * t
            if cost < min_cost:
                min_cost = cost
                goal = (x, y)

        # DEBUG: Publish goal point
        self.mark_point(*goal, [0, 1, 0])

        # TODO: Send goal to move_base
                        
if __name__ == '__main__':
    rospy.init_node('explore')
    node = Explorer()
    rospy.spin()
