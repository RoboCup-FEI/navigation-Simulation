#!/usr/bin/env python3
# Robot nasleduje cloveka: z polohy, ktoru deteguje dynamic_obstacle_predictor
# (/detected_obstacle_pose, najblizsi pohyblivy track), posiela Nav2 ciele na
# /goal_pose. Ciel je follow_distance pred clovekom na spojnici s robotom, natoceny
# k nemu; bt_navigator novym cielom nahradi predosly. Ked je robot blizsie nez
# follow_distance + tolerance alebo clovek nebol videny dlhsie nez lost_timeout,
# nove ciele neposiela a Nav2 dojde k poslednemu.
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        self.follow_distance = self.declare_parameter('follow_distance', 1.0).value
        self.tolerance = self.declare_parameter('tolerance', 0.3).value
        self.min_goal_shift = self.declare_parameter('min_goal_shift', 0.25).value
        self.lost_timeout = self.declare_parameter('lost_timeout', 2.0).value
        self.frame = self.declare_parameter('frame', 'odom').value
        self.person = None
        self.person_time = None
        self.robot = None
        self.last_goal = None
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(PoseStamped, '/detected_obstacle_pose', self.on_person, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.5, self.update)

    def on_person(self, msg):
        self.person = (msg.pose.position.x, msg.pose.position.y)
        self.person_time = self.get_clock().now()

    def on_odom(self, msg):
        self.robot = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def update(self):
        if self.person is None or self.robot is None:
            return
        if (self.get_clock().now() - self.person_time).nanoseconds * 1e-9 > self.lost_timeout:
            return
        dx, dy = self.person[0] - self.robot[0], self.person[1] - self.robot[1]
        dist = math.hypot(dx, dy)
        if dist < self.follow_distance + self.tolerance:
            return
        gx = self.person[0] - self.follow_distance * dx / dist
        gy = self.person[1] - self.follow_distance * dy / dist
        if self.last_goal and math.hypot(gx - self.last_goal[0], gy - self.last_goal[1]) < self.min_goal_shift:
            return
        yaw = math.atan2(dy, dx)
        goal = PoseStamped()
        goal.header.frame_id = self.frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = gx, gy
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_pub.publish(goal)
        self.last_goal = (gx, gy)


def main():
    rclpy.init()
    rclpy.spin(PersonFollower())


if __name__ == '__main__':
    main()
