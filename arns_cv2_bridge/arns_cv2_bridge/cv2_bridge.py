import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
import cv2

class Frame_Capture(Node):

    def __init__(self):
        super().__init__('detect_ball')

        self.get_logger().info('Looking for the ball...')
        self.image_sub = self.create_subscription(Image,"/image_raw",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.isFinish = False

        self.bridge = CvBridge()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            if self.isFinish == False:
               if cv2.waitKey(10) & 0xFF == ord('q'):
                  self.isFinish = True
                  cv2.destroyAllWindows()
               else:
                  
                  cv2.imshow('Fall Detection', cv_image)
                  # keypoints_norm, out_image, tuning_image = proc.find_circles(cv_image, self.tuning_params)

                  img_to_pub = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                  img_to_pub.header = data.header
                  self.image_out_pub.publish(img_to_pub)

        except CvBridgeError as e:
            print(e)  


def main(args=None):

    rclpy.init(args=args)

    frame_capture = Frame_Capture()
    rclpy.spin(frame_capture)

    frame_capture.destroy_node()
    rclpy.shutdown()