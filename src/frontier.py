import math
from geometry_msgs.msg import Point, Pose 
from src.roomba.src.util import getHeading


class Frontier:
    alpha = 0.3  # Distance to frontier
    beta = 0.05  # Size of frontier (decreases cost)
    gamma = 0.9  # Angle to frontier

    def __init__(self):
        self.size = 0
        self.distance = math.inf
        self.centre = Point(0, 0, 0)
        self.cost = math.inf
        self.points = []
        self.reference = Pose()

    def get_cost(self) -> float:
        if self.cost == math.inf:
            rt = getHeading(self.reference.orientation)
            ft = math.atan2(self.centre.y, self.centre.x)
            dt = abs(ft - rt)
            self.cost = self.alpha * self.distance - self.beta * self.size + self.gamma * dt
        return self.cost
