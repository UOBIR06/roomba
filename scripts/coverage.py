#!/usr/bin/python3

import os
import rospy
import datetime
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from roomba import util


class Coverage(object):
    def __init__(self):
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.vel_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, self.odom_callback)
        self.start_time = rospy.get_time()
        self.map_data = []
        self.idle_time = 0
        self.was_idle = True
        self.last_idle = rospy.get_time()
        self.last_pose = Pose()

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        active = abs(self.last_pose.position.x - pose.position.x) > 0 \
                 or abs(self.last_pose.position.y - pose.position.y) > 0 \
                 or abs(util.getHeading(pose.orientation) - util.getHeading(self.last_pose.orientation)) > 0

        self.last_pose = pose
        if active and self.was_idle:  # Started moving again
            self.was_idle = False
            self.idle_time += rospy.get_time() - self.last_idle
        elif not active and not self.was_idle:  # Stopped moving
            self.was_idle = True
            self.last_idle = rospy.get_time()

    def map_callback(self, msg: OccupancyGrid):
        count = sum([i >= 0 for i in msg.data])
        time = rospy.get_time()
        self.map_data.append((time, count))

    def flush(self):
        total = rospy.get_time() - self.start_time
        rospy.loginfo(f'!!! STATS !!!\n\tIdle time: {self.idle_time}\n\tTotal time: {total}')

        output = ''
        for t, d in self.map_data:
            output += f'{t},{d}\n'

        date = datetime.datetime.now()
        filename = date.strftime("%y-%m-%d_%H-%M-%S")
        with open(os.path.join(util.HOME_ROOT, f'data/logs/{filename}.csv'), 'w') as f:
            f.write(output)
            rospy.loginfo(f'Wrote output to {f.name}')


if __name__ == "__main__":
    rospy.init_node('coverage', anonymous=True)
    node = Coverage()
    rospy.wait_for_message('/roomba/map_ready', OccupancyGrid)  # Do nothing with message
    node.flush()
