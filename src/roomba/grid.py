import math
import rospy
from typing import Optional
from roomba.heap import Heap
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid


class Grid(object):
    FREE_CELL = 0
    FULL_CELL = 100
    UKNW_CELL = -1

    def __init__(self, g: OccupancyGrid):
        self.data = g.data
        self.info = g.info
        self.width = g.info.width
        self.height = g.info.height
        self.origin = g.info.origin.position
        self.resolution = g.info.resolution

    def check_bounds(self, i: int) -> bool:
        return 0 <= i <= self.width * self.height

    def get_index(self, p: Point) -> Optional[int]:
        """Convert point to index."""
        x = int((p.x - self.origin.x) / self.resolution + 0.5)
        y = int((p.y - self.origin.y) / self.resolution + 0.5)
        i = x + y * self.width
        if self.check_bounds(i):
            return i

    def get_point(self, i: int) -> Point:
        """Convert index to point."""
        y, x = divmod(i, self.width)
        x = self.origin.x + (x + 0.5) * self.resolution
        y = self.origin.y + (y + 0.5) * self.resolution
        return Point(x, y, 0)

    def nhood_4(self, i: int) -> list:
        """Make a list of in-bound 4-neighbours of a given index."""
        n = []
        if i > self.width * self.height - 1:
            return n  # Out of bounds
        if i % self.width > 0:
            n.append(i - 1)
        if i % self.width < self.width - 1:
            n.append(i + 1)
        if i >= self.width:
            n.append(i - self.width)
        if i < self.width * (self.height - 1):
            n.append(i + self.width)
        return n

    def nhood_8(self, i: int) -> list:
        """Make a list of 8-neighbours of a given index."""
        n = self.nhood_4(i)
        if i > self.width * self.height - 1:
            return n  # Out of bounds
        if i % self.width > 0 and i >= self.width:
            n.append(i - 1 - self.width)
        if i % self.width > 0 and i < self.width * (self.width - 1):
            n.append(i - 1 + self.width)
        if i % self.width < self.width - 1 and i >= self.width:
            n.append(i + 1 - self.width)
        if i % self.width < self.width - 1 and i < self.width * (self.height - 1):
            n.append(i + 1 + self.width)
        return n

    def get_cell(self, i: int) -> int:
        """Get cell occupancy value."""
        if self.check_bounds(i):
            return self.data[int(i)]
        else:
            pt = self.get_point(i)
            rospy.logerr(f'Index {i} ({pt.x}, {pt.y}) is out of bounds.')
            return self.UKNW_CELL

    def is_free(self, i: int) -> bool:
        return self.get_cell(i) == self.FREE_CELL

    def is_full(self, i: int) -> bool:
        return self.get_cell(i) == self.FULL_CELL

    def is_unknown(self, i: int) -> bool:
        return self.get_cell(i) == self.UKNW_CELL

    # !!! Jump Point Search (JPS) methods !!! #

    def distance(self, i: int, j: int) -> tuple:
        """Distance between two cells."""
        iy, ix = divmod(i, self.width)
        jy, jx = divmod(j, self.width)
        return ix - jx, iy - jy

    def manhattan(self, i: int, j: int) -> float:
        """Manhattan distance between two cells."""
        dx, dy = self.distance(i, j)
        return abs(dx) + abs(dy)

    def free(self, x: int, y: int) -> bool:
        """JPS helper using coordinates."""
        return 0 <= x < self.width and \
               0 <= y < self.height and \
               self.data[x + y * self.width] == self.FREE_CELL

    def index(self, x: int, y: int) -> int:
        """Coordinates to index."""
        return x + y * self.width

    def prune(self, i: int, p: Optional[int]) -> list:
        """JPS helper for finding pruned neighbours."""

        # No parent, return all neighbours
        if p is None:
            return self.nhood_8(i)

        # Initialize variables
        n = []
        y, x = divmod(i, self.width)
        dx, dy = self.distance(i, p)

        if dx != 0 and dy != 0:  # Diagonal move
            if self.free(x, y + dy):
                n.append(self.index(x, y + dy))
            if self.free(x + dx, y):
                n.append(self.index(x + dx, y))
            if self.free(x + dx, y + dy):
                n.append(self.index(x + dx, y + dy))
            if not self.free(x - dx, y):
                n.append(self.index(x - dx, y + dy))
            if not self.free(x, y - dy):
                n.append(self.index(x + dx, y - dy))
        else:  # Straight move
            if dx == 0:
                if self.free(x, y + dy):
                    n.append(self.index(x, y + dy))
                if not self.free(x + 1, y):
                    n.append(self.index(x + 1, y + dy))
                if not self.free(x - 1, y):
                    n.append(self.index(x - 1, y + dy))
            else:
                if self.free(x + dx, y):
                    n.append(self.index(x + dx, y))
                if not self.free(x, y + 1):
                    n.append(self.index(x + dx, y + 1))
                if not self.free(x, y - 1):
                    n.append(self.index(x + dx, y - 1))
        return n

    def jump(self, i: int, p: int, g: int) -> Optional[int]:
        """JPS helper for finding jump points."""

        # Not free space, stop
        if not self.is_free(i):
            return None

        # At goal, done
        if i == g:
            return g

        # Initialize variables
        y, x = divmod(i, self.width)
        dx, dy = self.distance(i, p)

        # Check for forced neighbours...
        if dx != 0 and dy != 0:  # ...along diagonal
            if (self.free(x - dx, y + dy) and not self.free(x - dx, y)) or \
               (self.free(x + dx, y - dy) and not self.free(x, y - dy)):
                return i
            if self.jump(self.index(x + dx, y), i, g) or \
               self.jump(self.index(x, y + dy), i, g):
                return i
        else:  # ...along straight
            if dx != 0:
                if (self.free(x + dx, y + 1) and not self.free(x, y + 1)) or \
                   (self.free(x + dx, y - 1) and not self.free(x, y - 1)):
                    return i
            else:
                if (self.free(x + 1, y + dy) and not self.free(x + 1, y)) or \
                   (self.free(x - 1, y + dy) and not self.free(x - 1, y)):
                    return i
        return self.jump(self.index(x + dx, y + dy), i, g)

    def find_path(self, start: int, goal: int) -> list:
        """Find a path between two points on the map."""
        # References:
        # 1. https://tilde.team/~kiedtl/blog/astar/
        # 2. https://www.redblobgames.com/pathfinding/grids/algorithms.html
        # 3. https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html
        # 4. http://users.cecs.anu.edu.au/~dharabor/data/papers/harabor-grastien-aaai11.pdf
        # 5. https://github.com/qiao/PathFinding.js

        # Initialize variables
        heap = Heap()
        heap.put(start, self.manhattan(start, goal))
        g_score = {start: 0}
        came_from = {start: None}
        const = math.sqrt(2) - 1

        while not heap.empty():
            i = heap.get()

            # Reconstruct path and return
            if i == goal:
                node = goal
                path = []
                while node in came_from:
                    pt = self.get_point(node)
                    path.append(pt)
                    node = came_from[node]
                path.reverse()
                return path

            # Look at pruned neighbours
            for n in self.prune(i, came_from[i]):
                j = self.jump(n, i, goal)
                if j:
                    # Calculate octile distance to jump point
                    dx, dy = self.distance(j, i)
                    dx = abs(dx)
                    dy = abs(dy)
                    score = g_score[i] + (const * dx + dy if dx < dy else const * dy + dx)

                    # Update with better score
                    if score < g_score.get(j, math.inf):
                        came_from[j] = i
                        g_score[j] = score
                        if not heap.has(j):
                            heap.put(j, score + self.manhattan(j, i))
        return []  # No path found!
