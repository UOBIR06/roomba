import rospy
from geometry_msgs.msg import Point, Quaternion, Vector3
from visualization_msgs.msg import Marker


class Viz(object):
    def __init__(self):
        self.mark_pub = rospy.Publisher('/marker', Marker, queue_size=1000)
        self.point_id = 0
        self.line_id = 0

    def clear_marks(self):
        """Clear RViz markers."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time()
        m.ns = 'point'
        m.action = 3  # Delete all objects in namespace
        self.mark_pub.publish(m)
        self.point_id = 0

    def clear_lines(self):
        """Clear RViz lines."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time()
        m.ns = 'point'
        m.action = 3  # Delete all objects in namespace
        self.mark_pub.publish(m)
        self.point_id = 0

    def reset(self):
        """Clear everything."""
        self.clear_marks()
        self.clear_lines()

    def mark_point(self, x: float, y: float, color=(1, 1, 1), scale=0.15):
        """Place a marker in RViz using '/marker' topic."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time()

        m.ns = 'point'
        m.id = self.point_id
        self.point_id += 1

        m.type = 1  # Use a cube
        m.action = 0  # Add/Modify mark

        m.pose.position = Point(x, y, 0)
        m.pose.orientation = Quaternion(0, 0, 0, 1)
        m.scale = Vector3(scale, scale, 0.01)

        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1

        m.lifetime = rospy.Duration(0)
        self.mark_pub.publish(m)

    def mark_frontiers(self, frontiers: list):
        """Convenience method to draw frontiers"""
        for f in frontiers:
            for pt in f.points:
                self.mark_point(pt.x, pt.y, (0, 0, 1), 0.05)
            self.mark_point(f.centre.x, f.centre.y, (0, 1, 0), 0.10)

    def draw_lines(self, points: list, color=(1, 1, 1), scale=0.1):
        """Draw a line in RViz."""
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time()

        m.ns = 'line'
        m.id = self.line_id
        self.line_id += 1

        m.type = 4  # Use line strip
        m.action = 0  # Add/Modify mark
        m.scale.x = scale

        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1

        m.points = points
        m.lifetime = rospy.Duration(0)
        self.mark_pub.publish(m)
