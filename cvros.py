import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2 as cv
from sensor_msgs.msg import Image
import math

def generate_gradient(rows, cols):
    # Generate vertical gradient ranging from -1 to 1
    lin = math.sin(math.radians(55/2)) * np.linspace(-1, 1, rows)
    grad = np.transpose(np.tile(lin, (cols, 1)))
    return grad

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.subscriber = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.subscriber2 = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.br = CvBridge()
        self.image = np.array([])
        self.hasImage = False
        self.i = 0

    def depth_callback(self, msg):
        im = self.br.imgmsg_to_cv2(msg)
        rows, cols = im.shape
        grad = generate_gradient(rows, cols)
        yHeight = im*grad
        mask = cv.inRange(yHeight, -30, 30) 
        thresh2 = cv.inRange(im, 10, 5000)
        mask2 = cv.convertScaleAbs(cv.bitwise_and(mask, mask, mask=thresh2))
        if self.hasImage:
            cv.imshow("orig", cv.convertScaleAbs(im, alpha=0.1))
            cv.imshow("height", cv.convertScaleAbs(yHeight,alpha=0.1))
            cv.imshow("thresh", cv.bitwise_and(self.image, self.image, mask=mask2))
        cv.waitKey(1)

    def color_callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)
        self.hasImage = True



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

