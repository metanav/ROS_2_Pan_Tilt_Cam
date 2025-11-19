import rclpy
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from vision_msgs.msg import Detection2DArray

class ImageBboxNode(Node):

    def __init__(self):
        super().__init__('image_bbox_node')

        qos = QoSProfile(depth=10)
        self.sub_img = Subscriber(self, Image, '/camera/image_raw')
        self.sub_dets = Subscriber(self, Detection2DArray, '/edge_impulse/detections')
        self.pub_img = self.create_publisher(Image, '/edge_impulse/img_bbox', 10)

        self.cvbridge = CvBridge()
        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.sub_img, self.sub_dets], queue_size, max_delay)
        self.time_sync.registerCallback(self.SyncCallback)
        self.get_logger().info('Image BBox node Started')
  
    def SyncCallback(self, img_msg, detections_msg):
        img = self.cvbridge.imgmsg_to_cv2(img_msg, 'passthrough')
        img_bbox = img.copy()

        scale = 240/224
        x_off = (320-240)/2
        y_off = (240-240)/2

        for det2D in detections_msg.detections:
            w = det2D.bbox.size_x
            h = det2D.bbox.size_y 
            x1 = int((det2D.bbox.center.position.x - w/2) * scale + x_off)
            y1 = int((det2D.bbox.center.position.y - h/2) * scale + y_off)
            x2 = int((det2D.bbox.center.position.x + w/2) * scale + x_off)
            y2 = int((det2D.bbox.center.position.y + h/2) * scale + y_off)
            #img_bbox = cv2.rectangle(img_bbox, (x1, y1), (x2, y2), (255, 0, 0), 2)
            img_bbox = cv2.rectangle(img_bbox, (x1, y1), (x2, y2), (0, 255, 255), 2)

        img_bbox_msg = self.cvbridge.cv2_to_imgmsg(img_bbox, 'rgb8')
        self.pub_img.publish(img_bbox_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        img_bbox_node = ImageBboxNode()
        rclpy.spin(img_bbox_node)
    except KeyboardInterrupt:
        pass
    finally:
        img_bbox_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
