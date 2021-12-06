#!/usr/bin/python3

import os
import rospy
import datetime
from nav_msgs.msg import OccupancyGrid
from roomba import util


class Coverage(object):
    def __init__(self):
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.data = []

    def map_callback(self, msg: OccupancyGrid):
        count = sum([i >= 0 for i in msg.data])
        time = rospy.get_time()
        self.data.append((time, count))

    def flush(self):
        output = ''
        for t, d in self.data:
            output += f'{t},{d}\n'

        date = datetime.datetime.now()
        filename = date.strftime("%y-%m-%d_%H-%M-%S")
        with open(os.path.join(util.HOME_ROOT, f'data/{filename}.csv'), 'w') as f:
            f.write(output)
            rospy.loginfo(f'Wrote output to {f.name}')


if __name__ == "__main__":
    rospy.init_node('coverage', anonymous=True)
    node = Coverage()
    rospy.spin()
    node.flush()
