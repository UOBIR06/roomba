#!/usr/bin/python
"""
@author yanrong
@date 04/12/2021
"""
from dataclasses import dataclass

import geometry_msgs.msg
from sensor_msgs.msg import Image as SensorImage


@dataclass
class RoomInfo:
    centre: geometry_msgs.msg.Point32 = None  # provides the (x,y)-coordinates of the room centre
    # indices: list = None                      # 0 for free, 255 for unavailable
    id: int = 0                            # 0 for unavailable, 1-n is room number, 255 for free
                                            # (255 can be used for testing coverage)
    area: float = 0.0                         # percentage
    img: SensorImage = None  # 0 for free, 255 for unavailable
    cleaned: int = 0  # 0: not cleaned, 1: cleaned

