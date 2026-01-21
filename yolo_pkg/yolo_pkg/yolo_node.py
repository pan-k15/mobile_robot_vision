import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Image,
            '/yolo/image_annotated',
            10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame, verbose=False)

        annotated = frame
        for r in results:
            annotated = r.plot()

        self.pub.publish(
            self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        )

        cv2.imshow("YOLO", annotated)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
