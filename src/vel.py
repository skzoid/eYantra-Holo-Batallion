#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from PID_controller import HolonomicPIDController
from hb_interfaces.msg import Poses2D
import numpy as np
import math
import matplotlib.pyplot as plt


class vel(Node):
    def __init__(self):
        super().__init__("vel_node")

        self.pose_sub = self.create_subscription(
            Poses2D,
            '/bot_pose',
            self.pose_cb,
            10
        )
        self.robot_id = 2
        # self.controller = HolonomicPIDController(bot_id = self.robot_id , dock_or_drop = False)
        # self.goals = [(1000,225)]
        # self.controller.set_goals(self.goals)
        self.current_pose = [1570 ,225 , np.pi/2]
        self.prev_pose = [1570 , 1000 , np.pi/2]
        self.last_time = 0
        self.timer = self.create_timer(0.5 , self.timer_cb)



    def pose_cb(self, msg):

        for pose in msg.poses:
            if pose.id == self.robot_id:
                yaw_in_degrees = pose.w
                yaw_in_radians = math.radians(yaw_in_degrees)
                self.current_pose = [pose.x, pose.y, yaw_in_radians]
                #self.calculate()

                return
            
    def calculate(self):
        x,y,yaw = tuple(self.current_pose)
        x_prev , y_prev , yaw_prev = tuple(self.prev_pose)

        vel_x = x - x_prev
        vel_y = y - y_prev

        now = self.get_clock().now().nanoseconds/1e9

        dt = now - self.last_time

        vel_x /= dt
        vel_y /= dt

        print(f"vel_x : {vel_x:.3f} , vel_y : {vel_y:.3f}")

        self.prev_pose = self.current_pose.copy()
        self.last_time = now

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds/1e9
        dt = now - self.last_time
        
        x,y,yaw = tuple(self.current_pose)
        x_prev , y_prev , yaw_prev = tuple(self.prev_pose)

        vel_x = x - x_prev
        vel_y = y - y_prev





        vel_x /= dt
        vel_y /= dt

        print(f"vel_x : {vel_x:.3f} , vel_y : {vel_y:.3f}")

        self.prev_pose = self.current_pose.copy()
        self.last_time = now





def main(args=None):
    rclpy.init(args=args)
    controller = vel()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
