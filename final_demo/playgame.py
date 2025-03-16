# playgame.py

#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node

from final_demo.KinematicChain import KinematicChain
from final_demo.TransformHelpers import *
from final_demo.robot_helpers import ik_solve, gravity_comp
from final_demo.chess_helpers import is_legal, find_best_move

from interfaces.msg import Segment, SegmentArray, ChessState
from std_msgs.msg import String, Bool

from stockfish import Stockfish

stockfish = Stockfish(path="/home/robot/robotws/src/final_demo/stockfish/stockfish-ubuntu-x86-64", parameters={"Threads": 4, "Minimum Thinking Time": 100})

RATE = 100.0  # Hz

ROBOT = 0
HUMAN = 1

class GameNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.starttime = self.get_clock().now()

        # Initialize kinematic chain for inverse kinematics
        self.chain = KinematicChain(self, 'world','tip',['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'])

        # Initialize publisher for game_instr
        self.seg_pub = self.create_publisher(SegmentArray, '/game_instr/segment_array', 5)
        self.fen_pub = self.create_publisher(Bool, 'game_instr/fen_request', 1)

        # Subscription for chess state
        self.sub_chess_state = self.create_subscription(ChessState, 'board_detector/chess_state', self.update_chessboard, 1)
        self.turn = None
        self.indicator_position = []
        self.added_state = False
        self.reported_fen = False
        self.completed_time = time.time()
        self.board_dict = {}

        # publisher to request FEN state
        self.fen_sub = self.create_subscription(String, 'fen_detector/fen_position', self.fen_reported, 5)
        self.last_fen_time = time.time()
        self.task_finished = True

        # subscription to determine if the current task is complete
        self.task_sub = self.create_subscription(Bool, 'game_instr/task_finished', self.request_fen, 1)

        self.capture_position = [.3, .5, .03]
        self.q_promote = [.3, .5, .03]
        self.r_promote = [.3, .5, .03]
        self.b_promote = [.3, .5, .025]
        self.k_promote = [.3, .5, .025]

        self.heights = {"KING": .03, "QUEEN": .03, "BISHOP": .025, "ROOK": .025, "KNIGHT":.025, "PAWN":.025}

    def request_fen(self, msg):
        current_time = time.time()
        
        if msg.data and self.turn == ROBOT and (not self.reported_fen) and (current_time > self.completed_time + 5.0):
            self.get_logger().info("FEN requested")
            req = Bool()
            req.data = True
            self.fen_pub.publish(req)
            self.completed_time = current_time

            self.reported_fen = True
    
    def update_chessboard(self, msg):
        for square in msg.squares:
            self.board_dict[square.name.lower()] = [square.x, square.y]
        self.turn = msg.player_turn
        self.indicator_position = [msg.indicator_x, msg.indicator_y, 0.01]

    def fen_reported(self, msg):
        fen = msg.data
        self.get_logger().info(f"FEN detected: {fen}")

        if self.turn == ROBOT and self.task_finished:
            self.do_turn(fen)

        self.reported_fen = False
        self.task_finished = True

    def offset2(self, x, y):

        x_offset = (x + 0.293) / 0.401 * 7.0 # 0-7
        y_offset = (y - 0.213) / 0.4075 * 7.0 # 0-7

        x_offset = 0.02 / 7.0 * x_offset - 0.007 / 7.0 * y_offset * (x_offset / 7.0)
        y_offset = -0.012 / 7.0 * y_offset + 0.01 / 7.0 * y_offset * (x_offset / 7.0)
        z_offset = 0.05 / 7.0 * y_offset

        return [x_offset, y_offset, z_offset]
    
    def fake_turn(self):
        d = {ROBOT: "Robot", HUMAN: "Human"}
        self.get_logger().info(f"player turn: {d[self.turn]}")
        self.get_logger().info(f"indicator position: {self.indicator_position}")

        if self.turn == ROBOT:
            self.segments.append(
                Segment(
                    joint_angles = [0.0, -math.pi/2, 0.0, 0.0, 0.0, 0.0],
                    joint_vels = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    time = 5.0
                ),
            )

            self.move_indicator()

    def move_indicator(self) -> list[Segment]:

        ind_offset = self.offset2(self.indicator_position[0], self.indicator_position[1])
        x = self.indicator_position[0] + ind_offset[0]
        y = self.indicator_position[1] + ind_offset[1]
        z = self.indicator_position[2] + ind_offset[2]

        segments = [
            self.generate_segment([x, y, z], 5.0, False, "moving"),                 # go right above indicator
            self.generate_segment([x, y, z-0.01], 2.0, True, "grab indicator"), # close gripper
            self.generate_segment([x, y, z+0.01], 2.0, True, "moving"),             # move up
            self.generate_segment([x, y+0.25, z+0.02], 4.0, True, "moving"),        # move closer toward human
            self.generate_segment([x, y+0.25, z+0.02], 1.0, False, "release indicator"),    # release
            self.generate_segment([x, y+0.25, z+0.1], 2.0, False, "moving"),        # move up
        ]

        segments += [
            Segment(
                joint_angles = [math.pi/2.5, -3*math.pi/4, -3*math.pi/4, math.pi/2, 0.0, 0.0],
                joint_vels = [0.0] * 6,
                time = 5.0
            )
        ]

        return segments
    
    def generate_moves(self, pos1, pos2):
        x1, y1, z1 = pos1[0], pos1[1], pos1[2]
        x2, y2, z2 = pos2[0], pos2[1], pos2[2]

        segments = [
            self.generate_segment([x1, y1, .2], 3.0, False, "moving"),          
            self.generate_segment([x1, y1, z1], 3.0, False, "down to grab piece"),  
            self.generate_segment([x1, y1, z1], 2.0, True, "close gripper to grab piece"),   
            self.generate_segment([x1, y1, .2], 3.0, True, "moving"),          
            self.generate_segment([x2, y2, .2], 3.0, True, "moving"),        
            self.generate_segment([x2, y2, z2], 3.0, True, "down to release piece"),   
            self.generate_segment([x2, y2, z2], 2.0, False, "open gripper to release piece"),  
            self.generate_segment([x2, y2, .2], 3.0, False, "moving")
        ]

        return segments

    def generate_segment(self, xyz_target, seg_time, closed, id=""):
        """
        Moves end-effector to xyz_target in seg_time seconds.
        Also sets final gripper angle if closed is True/False.
        """

        target_joints = ik_solve(self.chain, np.array(xyz_target))

        if target_joints is None:
            self.get_logger().error(f"IK did not converge for target {xyz_target}!")
            return

        # Set the 6th joint to open or close if requested:
        if closed is True:
            target_joints[-1] = -.4
        elif closed is False:
            target_joints[-1] = -.15

        seg = Segment(joint_angles=target_joints,
                      joint_vels=[0.0]*6,
                      time=seg_time,
                      id=id)
        
        self.get_logger().info(f"Added segment => xyz={xyz_target}, closed={closed}, time={seg_time}s")
        return seg
    
    def do_turn(self, fen_position):

        self.task_finished = False

        move_array, piece_types = find_best_move(stockfish, fen_position)
        self.get_logger().info(f"Optimal move: {move_array}")
        self.get_logger().info(f"Pieces: {piece_types}")

        seg_array = SegmentArray()

        for i in range(len(move_array)):
            move = move_array[i]
            square1 = move[0]
            square2 = move[1]

            type1 = str(piece_types[i]).split("_")[1]
            height = self.heights[type1]

            pos1 = []
            pos2 = []
            if square2 == "capture":
                pos1 = self.board_dict[square1] + [height]
                pos2 = self.capture_position + [height]
            elif square2[1:] == "q_promote":
                pos1 = self.q_promote + [height]
                pos2 = self.board_dict[square1] + [height]
            elif square2[1:] == "r_promote":
                pos1 = self.r_promote + [height]
                pos2 = self.board_dict[square1] + [height]
            elif square2[1:] == "b_promote":
                pos1 = self.b_promote + [height]
                pos2 = self.board_dict[square1] + [height]
            elif square2[1:] == "k_promote":
                pos1 = self.k_promote  + [height]
                pos2 = self.board_dict[square1] + [height]
            else:
                pos1 = self.board_dict[square1] + [height]
                pos2 = self.board_dict[square2] + [height]

            offset1 = self.offset2(pos1[0], pos1[1])
            offset2 = self.offset2(pos2[0], pos2[1])
            
            pos1 += offset1
            pos2 += offset2

            seg_array.segments += self.generate_moves(pos1, pos2)
        
        seg_array.segments += self.move_indicator()

        self.completed_time = time.time()

        self.seg_pub.publish(seg_array)
        self.get_logger().info("Published segment array")

    def shutdown(self):
        self.get_logger().info("Shutting down node.")

        # Destroy subscriptions
        self.destroy_subscription(self.sub_chess_state)
        self.destroy_subscription(self.fen_sub)

        # Destroy publishers
        self.destroy_publisher(self.seg_pub)

def main(args=None):
    rclpy.init(args=args)
    node = GameNode('game_node')
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()
