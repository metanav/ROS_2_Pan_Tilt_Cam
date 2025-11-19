import rclpy
import cv2
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from edge_impulse_linux.image import ImageImpulseRunner

class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection_node')

        self.declare_parameter('resource_path', '')
        resource_path = self.get_parameter('resource_path').get_parameter_value().string_value
        model_file = os.path.join(resource_path, 'hands-linux-aarch64-qnn-v9.eim')
        self.cvbridge = CvBridge()

        best_effort_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 
        )

        #self.sub = self.create_subscription(Image, '/image', self.image_callback, qos_profile=best_effort_qos_profile)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub_dets = self.create_publisher(Detection2DArray, '/edge_impulse/detections', 10)
        self.runner = ImageImpulseRunner(model_file)
        model_info = self.runner.init()
        self.get_logger().info('Object detection node Started')


    def image_callback(self, msg):
        img = self.cvbridge.imgmsg_to_cv2(msg, 'rgb8')
        features, cropped = self.runner.get_features_from_image_auto_studio_settings(img)
        res = self.runner.classify(features)

        vision_msg_dets  = Detection2DArray()
        vision_msg_dets.header = msg.header

        for bb in res["result"]["bounding_boxes"]:
            det2D = Detection2D()
            objHyp = ObjectHypothesisWithPose()
            det2D.bbox.center.position.x = float(bb['x'] + bb['width']/2)
            det2D.bbox.center.position.y = float(bb['y'] + bb['height']/2)
            det2D.bbox.size_x = float(bb['width'])
            det2D.bbox.size_y = float(bb['height'])
            objHyp.hypothesis.class_id = str(bb['label'])
            objHyp.hypothesis.score = float(bb['value'])
            det2D.results.append(objHyp)
            vision_msg_dets.detections.append(det2D)

        self.pub_dets.publish(vision_msg_dets)
    
    def destroy_node(self):
        if self.runner:
            self.runner.stop()
        if rclpy.ok():
            self.get_logger().info("Object detection node shut down.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()

    try:
        rclpy.spin(object_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        object_detection_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()




