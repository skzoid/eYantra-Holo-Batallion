#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class img_publisher(Node):
    def __init__(self):
        super().__init__('pub_node')

        self.cap = cv2.VideoCapture(2)


        width  = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        self.pub = self.create_publisher(Image , '/camera/image_raw' , 10)
        self.timer = self.create_timer(0.04 , self.image_pub)
        self.bridge = CvBridge()


    def image_pub(self):
        ret , frame = self.cap.read()
        if not ret:
            self.get_logger().info("ret_unsuccessful")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            exit()

        
        ros_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(ros_msg)
        
    

def main(args=None):
    rclpy.init(args=args)
    pub = img_publisher()
    try:
        rclpy.spin(pub)
    except KeyboardInterrupt:
        pass
    finally:
        pub.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()