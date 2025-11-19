import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float64MultiArray
from hand_tracker.pid import PID

class HandTrackerNode(Node):

    def __init__(self):
        super().__init__('hand_tracker_node')
        self.image_size = 224, 224
        self.pan_pid = PID(kp=0.07, ki=0.0, kd=0.005, setpoint=self.image_size[0]/2)
        self.tilt_pid = PID(kp=0.07, ki=0.0, kd=0.005, setpoint=self.image_size[0]/2)
        self.current_pan = 90.0   
        self.current_tilt = 90.0 
        self.last_time = None
        self.max_speed = 80.0
        self.pub = self.create_publisher(Float64MultiArray, '/ptcam_robot_controller/commands', 10)
        self.sub = self.create_subscription(Detection2DArray, '/edge_impulse/detections', self.detections_callback, 10)
        self.get_logger().info('Hand Tracker Node Started')
  
    def detections_callback(self, detections_msg):
        bbox_center = None

        for det2D in detections_msg.detections:
            bbox_center = det2D.bbox.center.position
            if abs(bbox_center.x - self.image_size[0]/2) < 10:
                bbox_center.x = self.image_size[0]/2
            if abs(bbox_center.y - self.image_size[0]/2) < 10:
                bbox_center.y = self.image_size[0]/2

        current_time = self.get_clock().now().nanoseconds / 1e9    

        if bbox_center is not None:
            pan_control = self.pan_pid.compute(bbox_center.x, current_time)
            tilt_control = self.tilt_pid.compute(bbox_center.y, current_time)

            if self.last_time is None:
                self.last_time = current_time 

            dt = max((current_time - self.last_time), 0.001)
            self.last_time = current_time

            max_delta = self.max_speed * dt
            pan_delta = np.clip(pan_control, -max_delta, max_delta)
            tilt_delta = np.clip(tilt_control, -max_delta, max_delta)

            self.current_pan += pan_delta
            self.current_tilt += tilt_delta

            # Clamp to servo limits
            self.current_pan = np.clip(self.current_pan, 0.0, 180.0)
            self.current_tilt = np.clip(self.current_tilt, 45.0, 150.0)

            commands = Float64MultiArray()
            commands.data = [ 
                np.deg2rad(self.current_pan),  
                np.deg2rad(self.current_tilt)]
            self.pub.publish(commands)
        else:
            self.pan_pid.reset()
            self.tilt_pid.reset()

def main(args=None):
    rclpy.init(args=args)
    try:
        hand_tracker_node = HandTrackerNode()
        rclpy.spin(hand_tracker_node)
    except KeyboardInterrupt:
        pass
    finally:
        hand_tracker_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
