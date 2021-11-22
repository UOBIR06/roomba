#!/usr/bin/python3
import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import MapMetaData, OccupancyGrid
import tf
from tf.msg import tfMessage
import math

class Explorer(object):
    FREE_CELL = 0
    FULL_CELL = 100
    UKNW_CELL = -1

    def __init__(self):
        self.tf = tf.TransformListener()
        self.tf_sub = rospy.Subscriber('/tf', tfMessage, self.tf_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.pose_pub = rospy.Publisher('/estimatedpose', PoseStamped, queue_size=1)

        # TODO: Play with these values
        self.alpha = 0.1
        self.beta = 1

    def tf_callback(self, msg):
        pass

    def get_occ(self, x, y):
        if x < 0 or y < 0 or x >= self.info.width or y >= self.info.height:
            raise IndexError(f'Index <{x}, {y}> is out of bounds for map size <{self.info.width}, {self.info.height}>')
        return self.data[x + y * self.info.width]

    def seed_fill(self, x, y):
        # Initialize search
        queue = [ (x, y) ]
        cells = []

        while queue:
            # Visit cell
            x, y = queue.pop()
            self.visit[(x, y)] = 0 # Use zero index

            # Look at 8 neighbours
            is_frontier = False
            neighbours = []
            for dx in range(-1,2):
                for dy in range(-1,2):
                    if dx == dy == 0:
                        continue # current cell

                    nx = x + dx
                    ny = y + dy
                    if nx < 0 or ny < 0 or nx >= self.info.width or ny >= self.info.height:
                        continue # out of bounds
                    if self.visit.get((nx, ny), -1) >= 0:
                        continue # already visited

                    occ = self.data[nx + ny * self.info.width]
                    if occ == self.FULL_CELL:
                        continue # don't process occupied cells
                    if occ == self.FREE_CELL:
                        is_frontier = True # Touching a FREE_CELL
                    if occ == self.UKNW_CELL:
                        neighbours.append((nx, ny)) # Neighbour UKNW_CELL

                # Update frontier 
                if is_frontier:
                    cells.append((x, y))
                    queue.extend(neighbours)

            # Calculate centre
            n = len(cells)
            cx = 0
            cy = 0
            for x, y in cells:
                cx = cx + x
                cy = cy + y
            cx = cx / n
            cy = cy / n

            return n, (cx, cy)

    def map_callback(self, msg):
        self.head = msg.header
        self.info = msg.info
        self.data = msg.data

        if self.tf.canTransform('/base_link', '/map', rospy.Time()):

            # FIXME: This is so fiddly. Sometimes the estimated pose is
            # really good, most other times it's absolutely terrible.
            # I tried using another SLAM package, hector_mapping, which had
            # terrible issues with divergence. I hate that gmapping is the
            # best one we have so far. See commented bits in explore.launch
            p, q = self.tf.lookupTransform('/base_link','/map',rospy.Time())

            # DEBUG: Publish pose estimate
            estimate = PoseStamped()
            estimate.pose = Pose(Point(*p), Quaternion(*q))
            estimate.header.frame_id = 'map'
            self.pose_pub.publish(estimate)

            # FIXME: Because `p` is in map coordinates, relative to the origin,
            # it may have negative components. I don't know how to convert from
            # map to grid index properly. As it is, px & py may be negative and
            # the check below always triggers, so nothing ever happens.
            
            #using the px / py calculation from ros-planning github
            #the map scale has been left as 1, I assume our maps will all have this scale
            #but could be wrong
            px = (math.floor((p[0] - MapMetaData.origin[0]) /  0.5 )+ MapMetaData.resolution[0]/2)
            py = (math.floor((p[1] - MapMetaData.origin[1]) /  0.5 )+ MapMetaData.resolution[1]/2)
            # Sanity check, robot should be in free cell
            #px = int(p[0] / self.info.resolution + 0.5)
            #py = int(p[1] / self.info.resolution + 0.5)

            if self.data[px + py * self.info.width] != self.FREE_CELL:
                rospy.logerr(f'Pose <{p[0]}, {p[1]}> -> <{px}, {py}> is not free space!')
                return
            
            # Initialize search
            queue = [ (px, py, 0) ]
            frontiers = []
            self.visit = { (px, py) : 0 }

            while queue:
                # Visit cell
                x, y, i = queue.pop()
                self.visit[(x, y)] = i

                # Look at 8 neighbours
                for dx in range(-1,2):
                    for dy in range(-1,2):
                        if dx == dy == 0:
                            continue # current cell
                        
                        nx = x + dx
                        ny = y + dy
                        if nx < 0 or ny < 0 or nx >= self.info.width or ny >= self.info.height:
                            continue # out of bounds
                        if self.visit.get((nx, ny), -1) >= 0:
                            continue # already visited

                        occ = self.data[nx + ny * self.info.width]
                        if occ == self.FULL_CELL:
                            continue # don't process occupied cells
                        if occ == self.FREE_CELL:
                            queue.append((nx, ny, i + 1)) # add to queue
                        if occ == self.UKNW_CELL:
                            # Current cell is guaranteed to be FREE_CELL and so
                            # if it has an UKNW_CELL neighbour that's a frontier
                            n, c = self.seed_fill(nx, ny)
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

                # TODO: Set goal
                print(f'Goal: {goal}')
                        
if __name__ == '__main__':
    rospy.init_node('explore')
    node = Explorer()
    rospy.spin()
