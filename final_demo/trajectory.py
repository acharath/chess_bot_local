# trajectory.py

#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from final_demo.TransformHelpers import *
from final_demo.robot_helpers import Spline, gravity_comp
from std_msgs.msg import Bool

from interfaces.msg import Segment, SegmentArray

RATE = 100.0  # Hz

ROBOT = 0
HUMAN = 1

DELAY = 5.0

class TrajectoryNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.delay_time = time.time() + DELAY

        # Grab current position and effort
        self.position0 = self.grabfbk()
        self.actpos = self.position0.copy()
        self.curr_effort = None

        # Create publishers
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)
        while not self.count_subscribers('/joint_commands'):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Connected to /joint_commands.")
        self.task_pub = self.create_publisher(Bool, 'game_instr/task_finished', 1)

        # Create subscribers
        self.fbksub = self.create_subscription(JointState, '/joint_states', self.recvfbk, 10)
        self.segsub = self.create_subscription(SegmentArray, '/game_instr/segment_array', self.update_segments, 5)

        # Initialize start time
        self.starttime = self.get_clock().now()
        self.timer = self.create_timer(1.0/RATE, self.update)
        self.last_update_time = time.time()

        # Initialize waiting position and commands
        self.waiting_position = [math.pi/2.5, -3*math.pi/4, -3*math.pi/4, math.pi/2, 0.0, 0.0]
        # self.waiting_position = [math.pi/2.5, -3*math.pi/4, -3*math.pi/4, math.pi/2, 0.0, 0.0]
        self.segments = [
            Segment(
                joint_angles = [0.0, -math.pi/2, 0.0, 0.0, 0.0, 0.0],
                joint_vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                time = 5.0
            ),
            Segment(
                joint_angles = self.waiting_position,
                joint_vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                time = 5.0
            )
        ]

        # Keep track of last commanded position & velocity
        self.t_last_cmd = 0.0
        self.x_last_cmd = self.position0.copy()
        self.v_last_cmd = [0.0]*6

        # Active spline
        self.spline = None
        self.done_moving = False

        # Previous spline
        self.prev_spline = None

    # -----------------------------------------------------------'---------------
    # Grab feedback once at startup
    # --------------------------------------------------------------------------
    def grabfbk(self):
        def cb(msg):
            self.grabpos = list(msg.position)
            self.grabready = True

        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        return self.grabpos

    def recvfbk(self, msg):
        self.actpos = list(msg.position)
        self.curr_effort = list(msg.effort)

    def update_segments(self, msg):
        for seg in msg.segments:
            self.segments.append(seg)

    # --------------------------------------------------------------------------
    # Publisher for joint commands
    # --------------------------------------------------------------------------
    def sendcmd(self, pos, vel, eff=[]):
        """Publish the command as given, so the 6th joint is not overwritten."""
        eff = eff + [0.0]*(6 - len(eff))
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name         = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        cmd.position     = pos
        cmd.velocity     = vel
        cmd.effort       = eff
        self.cmdpub.publish(cmd)

    def request_fen(self):
        if len(self.segments) == 0 and self.done_moving:
            current_time = time.time()
            if current_time > self.last_update_time + 2.0:
                msg = Bool()
                msg.data = True
                self.task_pub.publish(msg)
                self.get_logger().info("Robot stopped at t=%0.2f" % current_time)
                self.last_update_time = current_time
    
    def detect_collision(self):
        # First, detect if the robot hit a piece when moving down to grab it:
        # self.get_logger().info(f"prev_spline: {self.prev_spline is not None}")
        # self.get_logger().info(f"spline: {self.spline is not None}")

        # self.get_logger().info(f"current effort: {self.curr_effort}")

        # if self.spline and self.spline.get_id() == "down to grab piece":
        #     self.get_logger().info(f"curr_effort: {self.curr_effort}")

        if self.prev_spline and self.spline and self.spline.get_id() == "down to grab piece":
            self.get_logger().info(f"curr_effort: {self.curr_effort}")
            if self.curr_effort[1] > 1.0:
                self.get_logger().info("collision detected going down")
                new_down_segment = self.spline.segment
                prev_segment = self.prev_spline.segment

                # Create a new segment by rotating the gripper by pi/2 degrees
                prev_joint_angles = prev_segment.joint_angles
                prev_joint_angles = [
                    prev_joint_angles[0],
                    prev_joint_angles[1],
                    prev_joint_angles[2],
                    prev_joint_angles[3],
                    prev_joint_angles[4] + math.pi/2,
                    prev_joint_angles[5],
                ]

                new_joint_angles = new_down_segment.joint_angles
                new_joint_angles = [
                    new_joint_angles[0],
                    new_joint_angles[1],
                    new_joint_angles[2],
                    new_joint_angles[3],
                    new_joint_angles[4] + math.pi/2,
                    new_joint_angles[5],
                ]

                prev_segment.joint_angles = prev_joint_angles
                new_down_segment.joint_angles = new_joint_angles

                self.segments = [prev_segment, new_down_segment] + self.segments
                self.spline = None
                self.done_moving = True 


    # --------------------------------------------------------------------------
    # Main control loop
    # --------------------------------------------------------------------------
    def update(self):

        now = self.get_clock().now()
        t   = (now - self.starttime).nanoseconds * 1e-9

        if time.time() < self.delay_time:
            return

        if self.spline:
            if (t - self.spline.t0 > self.spline.T):
                self.prev_spline = self.spline
                self.spline = None
                self.done_moving = True
        else:
            if len(self.segments) > 0:
                seg = self.segments.pop(0)

                if self.spline:
                    self.prev_spline = self.spline
                self.spline = Spline(t, self.x_last_cmd, self.v_last_cmd, seg)
                self.done_moving = False
            else:
                self.done_moving = True

        if self.spline and not self.done_moving:
            x_cmd, v_cmd = self.spline.evaluate(t)
        else:
            x_cmd = self.x_last_cmd
            v_cmd = [0.0]*6

        self.request_fen()

        self.x_last_cmd = x_cmd
        self.v_last_cmd = v_cmd

        # Gravity compensation
        tau = gravity_comp(x_cmd)

        self.detect_collision()

        # self.get_logger().info(f"act_pos: {self.actpos[-1]}")
        
        self.sendcmd(x_cmd, v_cmd, tau)

    def shutdown(self):
        self.get_logger().info("Shutting down node.")
        self.destroy_timer(self.timer)
        self.destroy_subscription(self.fbksub)
        self.destroy_publisher(self.cmdpub)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode('trajectory')
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()
