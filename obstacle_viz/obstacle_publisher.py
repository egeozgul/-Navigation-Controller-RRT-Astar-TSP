#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

RECT_OBSTACLES = [
    [-40,   -30,  12, 3],
    [ 20,   -35,   6, 3],
    [-45,    15,  10, 4],
    [  5,    25,   3, 2],
    [ -6,  -6.5,  12, 3],
    [-13.5, -23.0,12, 6],
]
OBSTACLE_Z_HEIGHT = 1.5
ROBOT_START = (-40.0, -40.0)
GOAL_POS    = ( 20.0,  10.0)
OBSTACLE_COLOR = (0.85, 0.25, 0.15, 0.90)
ROBOT_COLOR    = (0.20, 0.60, 1.00, 1.00)
GOAL_COLOR     = (0.10, 0.90, 0.30, 1.00)

def rect_to_cube(rect, z_height):
    x, y, w, h = rect
    return x + w/2.0, y + h/2.0, z_height/2.0, float(w), float(h), z_height

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        latched_qos = QoSProfile(depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, '/obstacles', latched_qos)
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_map_frame()
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info('Obstacle publisher ready — publishing to /obstacles')

    def _publish_map_frame(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id  = 'map'
        # Positions are transformed in software; map frame is the display frame.
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)

    def publish_markers(self):
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, rect in enumerate(RECT_OBSTACLES):
            cx, cy, cz, sx, sy, sz = rect_to_cube(rect, OBSTACLE_Z_HEIGHT)
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = cz
            m.pose.orientation.w = 1.0
            m.scale.x = sx
            m.scale.y = sy
            m.scale.z = sz
            m.color.r, m.color.g, m.color.b, m.color.a = OBSTACLE_COLOR
            array.markers.append(m)
        robot = Marker()
        robot.header.frame_id = 'map'
        robot.header.stamp = stamp
        robot.ns = 'robot'; robot.id = 0
        robot.type = Marker.CYLINDER; robot.action = Marker.ADD
        robot.pose.position.x = ROBOT_START[0]
        robot.pose.position.y = ROBOT_START[1]
        robot.pose.position.z = 0.5
        robot.pose.orientation.w = 1.0
        robot.scale.x = robot.scale.y = 1.0; robot.scale.z = 1.0
        robot.color.r, robot.color.g, robot.color.b, robot.color.a = ROBOT_COLOR
        array.markers.append(robot)
        goal = Marker()
        goal.header.frame_id = 'map'
        goal.header.stamp = stamp
        goal.ns = 'goal'; goal.id = 0
        goal.type = Marker.SPHERE; goal.action = Marker.ADD
        goal.pose.position.x = GOAL_POS[0]
        goal.pose.position.y = GOAL_POS[1]
        goal.pose.position.z = 0.5
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 1.5
        goal.color.r, goal.color.g, goal.color.b, goal.color.a = GOAL_COLOR
        array.markers.append(goal)
        self.pub.publish(array)

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
