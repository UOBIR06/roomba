import math
from geometry_msgs.msg import Pose
from pf_localisation.util import getHeading


class Frontier:
    def __init__(self, n: int, d: int, x: float, y: float, p: Pose):
        self.size = n
        self.path_distance = d
        self.centre_x = x
        self.centre_y = y

        # Cost parameters TODO: Tweak these
        self.alpha = 0.3    # Distance to frontier
        self.beta = 0.2     # Size of frontier (decreases cost)
        self.gamma = 0.8    # Angle to frontier

        # Calculate cost for given pose
        # TODO: Angle difference can't distinguish between behind the robot and in front
        heading = getHeading(p.orientation)
        dt = abs(heading - self.get_angle())
        self.cost = self.alpha * d - self.beta * n + self.gamma * dt

    def get_size(self) -> int:
        return self.size

    def get_distance(self) -> int:
        return self.path_distance

    def get_centre(self) -> tuple:
        return self.centre_x, self.centre_y

    def get_angle(self) -> float:
        return math.atan2(self.centre_y, self.centre_x)

    def get_cost(self) -> float:
        return self.cost

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        else:
            return False
