import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float64MultiArray

class HandFollowerNode(Node):

    def __init__(self):
        super().__init__('hand_follower_node')

        self.sub_dets = self.create_subscription(Detection2DArray, '/edge_impulse/detections', self.detections_callback, 10)
        self.pub_cmds = self.create_publisher(Float64MultiArray, '/ptcam_robot_controller/commands', 10)
        self.data = [0.0, 0.0]
  
    def detections_callback(self, detections_msg):
        for det2D in detections_msg.detections:
            w = det2D.bbox.size_x
            h = det2D.bbox.size_y 
            x = det2D.bbox.center.position.x 
            y = det2D.bbox.center.position.y 

            if x > 0 and x < 150:
                self.data[0] = 2.0944 # 120 degrees
            if x > 180 and x < 320:
                self.data[0] = 1.0472 # 60 degrees
            if y > 0 and y < 150:
                self.data[1] = 2.0944 # 120 degrees
            if y > 180 and y < 320:
                self.data[1] = 1.0472 # 60 degrees

 
        commands = Float64MultiArray()
        commands.data = self.data
        self.pub_cmds.publish(commands)

def main(args=None):
    rclpy.init(args=args)
    try:
        hand_follower_node = HandFollowerNode()
        rclpy.spin(hand_follower_node)
    except KeyboardInterrupt:
        pass
    finally:
        hand_follower_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
