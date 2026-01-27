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
    WAITING = 0
    SEARCHING = 1
    CONFIRMING_DETECTION = 2
    APPROACHING = 3
    KICKING = 4
    STOPPING = 5
    GOAL_WATCHING = 6

class PenaltyShooter(Node):
    def __init__(self):
        super().__init__('penalty_shooter')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.model_states_sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # HSV range for red color detection
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
        # State machine initialization
        self.state = State.WAITING
        self.start_time = time.time()
        self.kick_start_time = None
        
        # Image processing and detection stability variables
        self.ball_detected = False
        self.ball_error_x = 0
        self.ball_width = 0
        self.detection_counter = 0
        self.CONFIRMATION_FRAMES = 5 # Relaxed detection requirement
        
        # Blind kick memory variables
        self.last_detection_time = 0.0
        self.last_ball_width = 0
        self.BLIND_SPOT_WIDTH_THRESHOLD = 180 # Width threshold to assume ball is in blind spot

        self.get_logger().info("Initializing PenaltyShooter node with blind-spot handling...")
        self.timer = self.create_timer(0.1, self.state_machine_loop)

    def image_callback(self, msg):
        if self.state not in [State.SEARCHING, State.CONFIRMING_DETECTION, State.APPROACHING]:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, d = cv_image.shape
            
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = mask1 | mask2

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                if area > 100:
                    self.ball_detected = True
                    M = cv2.moments(largest_contour)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        self.ball_error_x = cx - w / 2
                    _, _, self.ball_width, _ = cv2.boundingRect(largest_contour)
                    
                    # Update last known position info
                    self.last_detection_time = time.time()
                    self.last_ball_width = self.ball_width
                else:
                    self.ball_detected = False
            else:
                self.ball_detected = False
        except Exception as e:
            self.get_logger().error(f"Failed in image callback: {e}")

        if not self.ball_detected:
            self.detection_counter = 0

    def model_states_callback(self, msg):
        if self.state == State.GOAL_WATCHING:
            try:
                ball_index = msg.name.index('red_ball')
                ball_pose = msg.pose[ball_index]
                ball_x = ball_pose.position.x
                ball_y = ball_pose.position.y

                if ball_x < 0.0:
                    if -1.0 < ball_y < 1.0:
                        self.get_logger().info("GOAL!!! The ball entered the net.")
                    else:
                        self.get_logger().info("MISS... The ball went wide.")
                    
                    stop_twist = Twist()
                    self.cmd_vel_pub.publish(stop_twist)
                    self.timer.cancel()
                    self.destroy_node()
                    rclpy.shutdown()
            except ValueError:
                pass
            except Exception as e:
                self.get_logger().error(f"Failed in model_states_callback: {e}")

    def state_machine_loop(self):
        twist = Twist()

        if self.state == State.WAITING:
            self.get_logger().info("State: WAITING (3-second mandatory hold)")
            if time.time() - self.start_time > 3.0:
                self.state = State.SEARCHING
        
        elif self.state == State.SEARCHING:
            self.get_logger().info("State: SEARCHING for the ball")
            if self.ball_detected:
                self.state = State.CONFIRMING_DETECTION
                self.detection_counter = 0
            else:
                twist.angular.z = 0.5
        
        elif self.state == State.CONFIRMING_DETECTION:
            if not self.ball_detected:
                self.get_logger().info("Lost sight of ball, returning to SEARCH.")
                self.state = State.SEARCHING
                return

            self.detection_counter += 1
            self.get_logger().info(f"State: CONFIRMING ({self.detection_counter}/{self.CONFIRMATION_FRAMES} frames)")

            if self.detection_counter >= self.CONFIRMATION_FRAMES and abs(self.ball_error_x) <= 5:
                self.get_logger().info("Ball detection confirmed and centered. Proceeding to APPROACH.")
                self.state = State.APPROACHING
            else:
                twist.angular.z = -float(self.ball_error_x) / 200.0

        elif self.state == State.APPROACHING:
            if not self.ball_detected:
                time_since_last_seen = time.time() - self.last_detection_time
                # Blind Kick Condition Check
                if time_since_last_seen < 1.0 and self.last_ball_width > self.BLIND_SPOT_WIDTH_THRESHOLD:
                    self.get_logger().info("ASSUMING BALL IN BLIND SPOT. Transitioning to KICK.")
                    self.state = State.KICKING
                    self.kick_start_time = self.get_clock().now()
                else:
                    self.get_logger().warn("LOST BALL during approach! Stopping and returning to SEARCH.")
                    self.state = State.SEARCHING
                    self.cmd_vel_pub.publish(Twist())
                return

            self.get_logger().info(f"State: APPROACHING (Ball width: {self.ball_width})")
            
            if self.ball_width > 200:
                self.state = State.KICKING
                self.kick_start_time = self.get_clock().now()
                self.get_logger().info("Ball is close. Transitioning to KICK.")
                return

            twist.angular.z = -float(self.ball_error_x) / 300.0
            twist.linear.x = 1.5
        
        elif self.state == State.KICKING:
            self.get_logger().info("State: KICKING")
            kick_duration = Duration(seconds=0.3)
            if self.get_clock().now() - self.kick_start_time < kick_duration:
                twist.linear.x = 1.5
            else:
                self.state = State.STOPPING

        elif self.state == State.STOPPING:
            self.get_logger().info("State: STOPPING. Robot stopped, watching for goal.")
            self.state = State.GOAL_WATCHING
        
        elif self.state == State.GOAL_WATCHING:
            pass

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    penalty_shooter = PenaltyShooter()
    try:
        rclpy.spin(penalty_shooter)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            if 'penalty_shooter' in locals() and not penalty_shooter._destroyed:
                penalty_shooter.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()