#!/usr/bin/python3

import rospy
import datetime
from nav_msgs.msg import OccupancyGrid


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

        date = datetime.date.today()
        filename = date.strftime("%y-%m-%d_%h-%M-%s")
        with open(f'~/catkin_ws/src/roomba/data/{filename}.csv', 'w') as f:
            f.write(output)
            rospy.loginfo(f'Wrote output to {f}')


if __name__ == "__main__":
    rospy.init_node('coverage', anonymous=True)
    node = Coverage()
    rospy.spin()
    node.flush()
