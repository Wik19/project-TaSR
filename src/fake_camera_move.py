#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import cv2
import numpy as np

class CameraControlNode(Node):
    def __init__(self):
        super().__init__('camera_control_node')
        
        # Publisher to the standard UR controller topic
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # UR5e Joint Names (Order is important!)
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Current "Target" positions for the 6 joints
        # [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
        self.current_goal = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # A standard "Home" pose

        self.get_logger().info("Fake Camera Node Started! Click the window to move the robot.")

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.current_goal
        point.time_from_start.sec = 2  # Take 2 seconds to move there
        
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sending Target: {self.current_goal}")

    def run_window(self):
        # Create a black image (500x500 pixels)
        img = np.zeros((500, 500, 3), dtype='uint8')
        
        # Draw a grid for visual reference
        cv2.line(img, (250, 0), (250, 500), (255, 255, 255), 1)
        cv2.line(img, (0, 250), (500, 250), (255, 255, 255), 1)
        cv2.putText(img, "CLICK TO MOVE", (170, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, "X-Axis: Rotate Base", (10, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(img, "Y-Axis: Move Shoulder", (300, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("Fake Camera Control", img)
        
        # Mouse Callback function
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                # Map X pixel (0 to 500) to Base Joint (-3.14 to 3.14 radians)
                # 0 -> +3.14 (Left), 500 -> -3.14 (Right)
                base_angle = np.interp(x, [0, 500], [3.14, -3.14])
                
                # Map Y pixel (0 to 500) to Shoulder Joint (-3.14 to 0 radians)
                # 0 -> -3.14 (Back), 500 -> 0 (Front)
                shoulder_angle = np.interp(y, [0, 500], [-3.14, 0.0])
                
                # Update the goal
                self.current_goal[0] = float(base_angle)
                self.current_goal[1] = float(shoulder_angle)
                
                # Send the command
                self.send_trajectory()

        cv2.setMouseCallback("Fake Camera Control", mouse_callback)
        
        # ROS 2 Spin loop mixed with OpenCV loop
        while rclpy.ok():
            cv2.waitKey(10)
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraControlNode()
    
    try:
        node.run_window()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()