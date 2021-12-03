import math
import os
import time
import sys
import rospkg

import cv2
import cv_bridge
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image as SensorImage


#global variables
HOME_ROOT = rospkg.RosPack().get_path('roomba')

def timed(fn):
    """ Decorator to time functions. For debugging time critical code """

    def timed(*args, **kwargs):
        t = time.time()
        print("[", fn, __name__, "]Start: ", t)
        ret = fn(*args, **kwargs)
        print("[", fn, __name__, "]End:", time.time(), " = = = ", time.time() - t)
        return ret

    return timed


def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.
    
    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0

    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)

    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # ----- Multiply new (heading-only) quaternion by the existing (pitch and bank) 
    # ----- quaternion. Order is important! Original orientation is the second 
    # ----- argument rotation which will be applied to the quaternion is the first 
    # ----- argument. 
    return multiply_quaternions(q_headingChange, q_orig)


def multiply_quaternions(qa, qb):
    """
    Multiplies two quaternions to give the rotation of qb by qa.
    
    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()

    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.
    
    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw


def setup_image(image_path):
    """ Credit to https://github.com/RethinkRobotics/intera_sdk """
    """
    Load the image located at the specified path

    @type image_path: str
    @param image_path: the relative or absolute file path to the image file

    @rtype: sensor_msgs/Image or None
    @param: Returns sensor_msgs/Image if image convertable and None otherwise
    """
    if not os.access(image_path, os.R_OK):
        rospy.logerr("Cannot read file at '{0}'".format(image_path))
        return None

    img_flip = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    # img.encoding = 'mono8'
    img = cv2.flip(img_flip, 0)
    # Return msg
    # for i in img:
    #     for j in i:
    #         img[i][j] = 0 if img[i][j] < 250 else 255

    # img = numpy.array([numpy.array([0 if j < 250 else 255 for j in i])for i in img])
    # img =

    # Isolate the areas where the color is black(every channel=0) and white (every channel=255)
    black = np.where(img[:, :] < 250)
    white = np.where(img[:, :] >= 250)

    # Turn img to black and white
    img[black] = (0,)
    img[white] = (255,)

    return cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="mono8")  # , encoding="mono8"

def grid_to_sensor_image(map: OccupancyGrid) -> SensorImage: #mat feed into Image
    # Convert it to an image
    data = bytes(map(lambda x: 255 if x >=250 else 0, map.data))

    # Stuff it inside a sensor_msgs/Image
    img = SensorImage()
    img.width = map.info.width
    img.height = map.info.height
    img.encoding = 'mono8'#'8UC1'
    img.is_bigendian = (sys.byteorder == 'big')
    img.step = img.width
    img.data = data
    return img
