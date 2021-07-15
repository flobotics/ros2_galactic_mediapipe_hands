import rclpy
from rclpy.node import Node

from std_msgs.msg import String
# from sensor_msgs.msg import Image
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError

import mediapipe as mp
import cv2
import numpy as np
from PIL import Image

# import ros2_galactic_mediapipe_hands_interfaces.msg.Hand
from ros2_galactic_mediapipe_hands_interfaces.msg import Hand


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.hand_image_publisher_ = self.create_publisher(sensor_msgs.msg.Image, 'hand_image', 10)
        self.hand_msg_publisher_ = self.create_publisher(Hand, 'hand_msg', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        
        self.camera_image_subscription_ = self.create_subscription(
            sensor_msgs.msg.Image,
            'image',
            self.listener_callback,
            10)
        self.camera_image_subscription_  # prevent unused variable warning
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        
        # self.cap = cv2.VideoCapture(0)
        
    def listener_callback(self, data):
        bridge = CvBridge()
        try:
            
            image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr('Converting Image Error. ' + str(e))
            return

        
        with self.mp_hands.Hands(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands:
            
            # while self.cap.isOpened():
            # success, image = self.cap.read()
            # if not success:
            #     # print("Ignoring empty camera frame.")
            #     # If loading a video, use 'break' instead of 'continue'.
            #     continue
            
            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            results = hands.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            msg = Hand()
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    print(
                              f'Index finger tip coordinates: (',
                              f'{hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x}, '
                              f'{hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y})'
                          )
                    
                    msg.index_tip_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x
                    msg.index_tip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y
                    msg.index_tip_z = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].z
                    
                    msg.index_dip_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_DIP].x
                    msg.index_dip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_DIP].y
                    msg.index_dip_z = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_DIP].z
                    
                    msg.index_pip_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].x
                    msg.index_pip_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].y
                    msg.index_pip_z = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].z
                    
                    msg.index_mcp_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].x
                    msg.index_mcp_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y
                    msg.index_mcp_z = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].z
                    # hand_image_msg.index_finger_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x
                    # hand_image_msg.index_finger_tip_y = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y
                    # hand_image_msg.index_finger_tip_z = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].z
                    self.mp_drawing.draw_landmarks(
                        image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            
            
            # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            # image = Image.fromarray(image.astype(np.uint8))
            self.hand_image_publisher_.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
            self.hand_msg_publisher_.publish(msg)
            # cv2.imshow('MediaPipe Hands', image)
            # cap.release()     
            

    def timer_callback(self):
        pass
            
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1


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