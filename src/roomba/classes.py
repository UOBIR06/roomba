#!/usr/bin/python
"""
@author yanrong
@date 04/12/2021
"""
from dataclasses import dataclass

from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image as SensorImage


@dataclass
class RoomInfo:
    centre: Point32 = Point32()  # provides the (x,y)-coordinates of the room centre
    centre_pixel: Point32 = Point32()  # centre in pixel
    # indices: list = None                      # 0 for free, 255 for unavailable
    id: int = 0                            # 0 for unavailable, 1-n is room number, 255 for free
                                            # (255 can be used for testing coverage)
    area: float = 0.0                         # percentage
    img: SensorImage = None  # 0 for free, 255 for unavailable
    cleaned: int = 0  # 0: not cleaned, 1: cleaned

