#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
import curses
import time

NUM_OF_SERVOS = 7
NUM_OF_CRANE_MOTOR = 2

class CraneTUI(Node):
    def __init__(self, stdscr):
        super().__init__('crane_tui')
        self.stdscr = stdscr
        self.crane_pub = self.create_publisher(Int32MultiArray, 'crane_control', 10)
        self.joint_pub = self.create_publisher(JointTrajectoryPoint, 'joint_angles', 10)
        self.joint_angles = [90.0] * NUM_OF_SERVOS
        self.crane_state = [0] * NUM_OF_CRANE_MOTOR
        self.selected = 0  # 0~6 for servo, 7/8 for crane motors

    def draw(self):
        self.stdscr.clear()
        max_y, max_x = self.stdscr.getmaxyx()
        required_lines = 4 + NUM_OF_SERVOS + NUM_OF_CRANE_MOTOR + 2
        if max_y < required_lines:
            self.stdscr.addstr(0, 0, f'Terminal too small! Need at least {required_lines} lines, current: {max_y}')
            self.stdscr.addstr(1, 0, 'Please resize your terminal window.')
            self.stdscr.refresh()
            return
        self.stdscr.addstr(0, 0, 'Micro-ROS Crane/Arm TUI Controller')
        self.stdscr.addstr(2, 0, 'Use ←/→ to select, ↑/↓ to change value, Enter to send, q to quit')
        for i in range(NUM_OF_SERVOS):
            sel = '>' if self.selected == i else ' '
            self.stdscr.addstr(4+i, 0, f'{sel} Servo {i+1} Angle: {self.joint_angles[i]:.1f}')
        for i in range(NUM_OF_CRANE_MOTOR):
            sel = '>' if self.selected == NUM_OF_SERVOS + i else ' '
            self.stdscr.addstr(4+NUM_OF_SERVOS+i, 0, f'{sel} Crane Motor {i+1} State: {self.crane_state[i]}')
        self.stdscr.refresh()

    def run(self):
        total_items = NUM_OF_SERVOS + NUM_OF_CRANE_MOTOR
        while True:
            self.draw()
            key = self.stdscr.getch()
            if key == ord('q'):
                self.joint_angles = [90.0] * NUM_OF_SERVOS
                self.crane_state = [0] * NUM_OF_CRANE_MOTOR
                self.send_signals()
                break

            if key in [curses.KEY_LEFT, ord('h')]:
                self.selected = (self.selected - 1) % total_items
            elif key in [curses.KEY_RIGHT, ord('l')]:
                self.selected = (self.selected + 1) % total_items
            elif key in [curses.KEY_UP, ord('k')]:
                if self.selected < NUM_OF_SERVOS:
                    self.joint_angles[self.selected] = min(180.0, self.joint_angles[self.selected] + 5)
                else:
                    idx = self.selected - NUM_OF_SERVOS
                    self.crane_state[idx] = min(1, self.crane_state[idx] + 1)
            elif key in [curses.KEY_DOWN, ord('j')]:
                if self.selected < NUM_OF_SERVOS:
                    self.joint_angles[self.selected] = max(0.0, self.joint_angles[self.selected] - 5)
                else:
                    idx = self.selected - NUM_OF_SERVOS
                    self.crane_state[idx] = max(-1, self.crane_state[idx] - 1)
            elif key in [curses.KEY_ENTER, 10, 13]:
                self.send_signals()

    def send_signals(self):
        # Publish crane_control signal (Int32MultiArray)
        crane_msg = Int32MultiArray()
        crane_msg.data = self.crane_state.copy()
        self.crane_pub.publish(crane_msg)
        self.get_logger().info(f'Sent crane_control: {self.crane_state}')
        # Publish joint_angles signal
        joint_msg = JointTrajectoryPoint()
        joint_msg.positions = self.joint_angles.copy()
        self.joint_pub.publish(joint_msg)
        self.get_logger().info(f'Sent joint_angles: {self.joint_angles}')

def main(stdscr):
    rclpy.init()
    node = CraneTUI(stdscr)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    curses.wrapper(main)
