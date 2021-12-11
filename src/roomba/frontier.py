import math
from geometry_msgs.msg import Point, Pose 
from roomba.util import getHeading


class Frontier:
    alpha = 1.0  # Distance to frontier
    beta = 0.5   # Size of frontier (decreases cost)
    gamma = 0.5  # Angle to frontier

    def __init__(self, r: Pose, s: int, d: float, c: Point, p: list):
        self.size = s
        self.distance = d
        self.centre = c
        self.points = p

        rt = getHeading(r.orientation)
        ft = math.atan2(self.centre.y, self.centre.x)
        self.angle = abs(ft - rt)

    def get_cost(self) -> float:
        return self.alpha * self.distance - self.beta * self.size + self.gamma * self.angle
