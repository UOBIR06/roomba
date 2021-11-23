#!/usr/bin/python3
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import MapMetaData, OccupancyGrid
import tf
from tf.msg import tfMessage
import math
from visualization_msgs.msg import Marker

class Explorer(object):
    FREE_CELL = 0
    FULL_CELL = 100
    UKNW_CELL = -1
    temp = 2

    def __init__(self):
        self.tf = tf.TransformListener()
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.pose_pub = rospy.Publisher('/estimatedpose', PoseStamped, queue_size=1)
        self.mark_pub = rospy.Publisher('/marker', Marker, queue_size=100)

        # TODO: Play with these values
        self.alpha = 0.1
        self.beta = 1

    def build_frontier(self, x, y):
        # Initialize search
        queue = [ (x, y) ]
        cx = 0
        cy = 0
        n = 0

        while queue:
            # Visit cell
            x, y = queue.pop()
            self.visit[(x, y)] = 0 # Use zero index
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
        return n, (cx, cy)

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

    def mark_start(self, x, y):
        self.mark_point(x, y, 0, 1, 1, 0)

    def mark_goal(self, x, y):
        self.mark_point(x, y, 1, 0, 1, 0)

    def mark_front(self, x, y):
        self.mark_point(x, y, self.temp, 0, 0, 1)
        self.temp += 1

    def mark_point(self, x, y, i, r, g, b):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time()

        m.ns = 'point'
        m.id = i

        m.type = 1 # Use a cube
        m.action = 0 # Add/Modify mark

        m.pose.position = Point(x, y, 0)
        m.pose.orientation = Quaternion(0, 0, 0, 1)
        m.scale = Vector3(0.15, 0.15, 0.01)

        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1

        m.lifetime = rospy.Duration(0)
        self.mark_pub.publish(m)

    def map_callback(self, msg):
        self.head = msg.header
        self.info = msg.info
        self.data = msg.data
        self.temp = 2

        # Check if transform exists
        if not self.tf.canTransform('base_link', 'map', rospy.Time()):
            return

        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.header.stamp = rospy.Time()
        p = self.tf.transformPose('map', p)
        
        # DEBUG: Publish pose estimate
        self.pose_pub.publish(p)

        # Convert world coordinates to map indices
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
        self.mark_start(wx, wy)
        
        # Initialize search
        # TODO: If I'm not using the index numbers, just use booleans instead
        queue = [ (px, py, 0) ]
        frontiers = []
        self.visit = {}

        while queue:
            # Visit cell
            x, y, i = queue.pop()
            self.visit[(x, y)] = i

            # Look at 4 neighbours
            for nx, ny in self.nhood_4(x, y):
                if self.visit.get((nx, ny), -1) >= 0:
                    continue # already visited

                occ = self.get_cell(nx, ny)
                if occ == self.FREE_CELL:
                    queue.append((nx, ny, i + 1)) # add to queue
                if occ == self.UKNW_CELL:
                    # Current cell is guaranteed to be FREE_CELL and so
                    # if it has an UKNW_CELL neighbour that's a frontier
                    n, c = self.build_frontier(nx, ny)
                    frontiers.append((n, c))

        # Calculate costs
        min_cost = math.inf
        goal = None
        for f in frontiers:
            n = f[0]
            x, y = f[1]
            dist = math.sqrt((x - px)**2 + (y - py)**2)
            cost = self.alpha * dist - self.beta * n
            if cost < min_cost:
                min_cost = cost
                goal = (x, y)

        # DEBUG: Publish goal point
        self.mark_goal(*goal)
                        
if __name__ == '__main__':
    rospy.init_node('explore')
    node = Explorer()
    rospy.spin()
