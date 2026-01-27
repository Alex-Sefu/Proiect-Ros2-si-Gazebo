#!/usr/bin/env python3
"""
ROS2 Node for a Penalty Kick Simulation using a TurtleBot3.

This node controls a TurtleBot3 to find a red ball, approach it, and kick it
towards a goal. It uses a state machine to manage its behavior, from searching
for the ball to detecting a goal or a miss.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from enum import Enum

class State(Enum):
    """Defines the states for the robot's state machine."""
    WAITING = 0
    SEARCHING = 1
    CONFIRMING_DETECTION = 2
    APPROACHING = 3
    KICKING = 4
    STOPPING = 5
    GOAL_WATCHING = 6

class PenaltyShooter(Node):
    """
    Controls the TurtleBot3 for the penalty kick task.
    
    Subscribes to camera and Gazebo model states to perceive its environment
    and publishes velocity commands to control the robot's movement.
    """
    def __init__(self):
        """Initializes the node, publishers, subscribers, and state machine."""
        super().__init__('penalty_shooter')

        # --- ROS2 Communications ---
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.model_states_sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Image Processing Parameters ---
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
        # --- State Machine Initialization ---
        self.state = State.WAITING
        self.start_time = time.time()
        self.kick_start_time = None
        
        # --- Detection Stability & Blind Spot Variables ---
        self.ball_detected = False
        self.ball_error_x = 0
        self.ball_width = 0
        self.detection_counter = 0
        self.CONFIRMATION_FRAMES = 5
        self.last_detection_time = 0.0
        self.last_ball_width = 0
        self.BLIND_SPOT_WIDTH_THRESHOLD = 180

        self.get_logger().info("PenaltyShooter node initialized.")
        self.timer = self.create_timer(0.1, self.state_machine_loop)

    def image_callback(self, msg: Image):
        """
        Processes raw image data from the camera to detect the red ball.
        
        Args:
            msg: The incoming sensor_msgs/Image message.
        """
        if self.state not in [State.SEARCHING, State.CONFIRMING_DETECTION, State.APPROACHING]:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, _ = cv_image.shape
            
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = mask1 | mask2

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 100:
                    self.ball_detected = True
                    M = cv2.moments(largest_contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        self.ball_error_x = cx - w / 2
                    _, _, self.ball_width, _ = cv2.boundingRect(largest_contour)
                    self.last_detection_time = time.time()
                    self.last_ball_width = self.ball_width
                else:
                    self.ball_detected = False
            else:
                self.ball_detected = False
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

        if not self.ball_detected:
            self.detection_counter = 0

    def model_states_callback(self, msg: ModelStates):
        """
        Monitors the ball's position to detect a goal or miss after kicking.
        
        Args:
            msg: The incoming gazebo_msgs/ModelStates message.
        """
        # --- Goal Detection Logic ---
        if self.state == State.GOAL_WATCHING:
            try:
                ball_index = msg.name.index('red_ball')
                ball_pose = msg.pose[ball_index]
                if ball_pose.position.x < 0.0:
                    if -1.0 < ball_pose.position.y < 1.0:
                        self.get_logger().info("GOAL!!! The ball entered the net.")
                    else:
                        self.get_logger().info("MISS... The ball went wide.")
                    
                    self.cmd_vel_pub.publish(Twist()) # Final stop command
                    self.timer.cancel()
                    self.destroy_node()
                    rclpy.shutdown()
            except ValueError:
                pass # Ball not found in model states yet
            except Exception as e:
                self.get_logger().error(f"Model state processing failed: {e}")

    def state_machine_loop(self):
        """The main control loop driven by a timer."""
        twist = Twist()

        # --- State: WAITING ---
        # Enforce a mandatory hold to allow sensors to initialize.
        if self.state == State.WAITING:
            if time.time() - self.start_time > 3.0:
                self.state = State.SEARCHING
        
        # --- State: SEARCHING ---
        # Rotate until the ball is first sighted.
        elif self.state == State.SEARCHING:
            if self.ball_detected:
                self.state = State.CONFIRMING_DETECTION
                self.detection_counter = 0
            else:
                twist.angular.z = 0.5
        
        # --- State: CONFIRMING_DETECTION ---
        # Require several consecutive frames of detection before approaching.
        elif self.state == State.CONFIRMING_DETECTION:
            if not self.ball_detected:
                self.state = State.SEARCHING
                return

            self.detection_counter += 1
            if self.detection_counter >= self.CONFIRMATION_FRAMES and abs(self.ball_error_x) <= 5:
                self.state = State.APPROACHING
            else:
                # Rotate to center the ball, no linear movement.
                twist.angular.z = -float(self.ball_error_x) / 200.0

        # --- State: APPROACH ---
        # Move towards the ball, handling the camera's blind spot.
        elif self.state == State.APPROACHING:
            # --- Blind Spot Logic ---
            if not self.ball_detected:
                time_since_last_seen = time.time() - self.last_detection_time
                if time_since_last_seen < 1.0 and self.last_ball_width > self.BLIND_SPOT_WIDTH_THRESHOLD:
                    self.state = State.KICKING
                    self.kick_start_time = self.get_clock().now()
                else:
                    self.state = State.SEARCHING
                    self.cmd_vel_pub.publish(Twist())
                return

            # If ball is close enough, transition to kick.
            if self.ball_width > 200:
                self.state = State.KICKING
                self.kick_start_time = self.get_clock().now()
                return

            # Standard approach: correct angle and move forward.
            twist.angular.z = -float(self.ball_error_x) / 300.0
            twist.linear.x = 1.5
        
        # --- State: KICKING ---
        # Execute a short, powerful forward burst.
        elif self.state == State.KICKING:
            kick_duration = Duration(seconds=0.3)
            if self.get_clock().now() - self.kick_start_time < kick_duration:
                twist.linear.x = 1.5
            else:
                self.state = State.STOPPING

        # --- State: STOPPING ---
        # Cease all movement and prepare to watch the ball.
        elif self.state == State.STOPPING:
            self.state = State.GOAL_WATCHING
        
        # --- State: GOAL_WATCHING ---
        # Passively wait for the model_states_callback to detect goal/miss.
        elif self.state == State.GOAL_WATCHING:
            pass

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    """Main function to initialize and run the ROS2 node."""
    rclpy.init(args=args)
    penalty_shooter = PenaltyShooter()
    try:
        rclpy.spin(penalty_shooter)
    except (SystemExit, KeyboardInterrupt):
        pass # Graceful shutdown on Ctrl+C or node-initiated shutdown
    finally:
        if rclpy.ok() and 'penalty_shooter' in locals() and not penalty_shooter._destroyed:
            penalty_shooter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
