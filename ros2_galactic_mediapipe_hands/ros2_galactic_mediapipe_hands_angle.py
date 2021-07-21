import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
import sensor_msgs
from cv_bridge import CvBridge, CvBridgeError

import mediapipe as mp
import cv2
import numpy as np
from PIL import Image

from tf2_ros import TransformBroadcaster , TransformStamped
# from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from math import sin, cos, pi
from rclpy.qos import QoSProfile

from ros2_galactic_mediapipe_hands_interfaces.msg import Hand

import math 
import time


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.hand_image_publisher_ = self.create_publisher(sensor_msgs.msg.Image, 'hand_image', 10)
        # self.hand_msg_publisher_ = self.create_publisher(Hand, 'hand_msg', 10)
        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        self.camera_image_subscription_ = self.create_subscription(
            sensor_msgs.msg.Image,
            'image',
            self.listener_callback,
            10)
        self.camera_image_subscription_  # prevent unused variable warning

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands


    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    #calculate angle between 3 points in 3d space
    def angle_2p_3d(self, a, b, c):       

        v1 = np.array([ a[0] - b[0], a[1] - b[1], a[2] - b[2] ])
        v2 = np.array([ c[0] - b[0], c[1] - b[1], c[2] - b[2] ])
    
        v1mag = np.sqrt([ v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] ])
        v1norm = np.array([ v1[0] / v1mag, v1[1] / v1mag, v1[2] / v1mag ])
    
        v2mag = np.sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2])
        v2norm = np.array([ v2[0] / v2mag, v2[1] / v2mag, v2[2] / v2mag ])
        res = v1norm[0] * v2norm[0] + v1norm[1] * v2norm[1] + v1norm[2] * v2norm[2]
        angle_rad = np.arccos(res)
    
        return math.degrees(angle_rad)


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

            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            results = hands.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            (iwidth,iheight,channels) = image.shape
            iwidth = 1
            iheight = 1

          
            joint_state = JointState()

          

            msg = Hand()
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # print(
                    #           f'Index finger tip coordinates: (',
                    #           f'{hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x}, '
                    #           f'{hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y})'
                    #       )
                    msg.wrist_x = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x
                    msg.wrist_y = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].y
                    msg.wrist_z = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].z

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

                    tilt = 0.
                    swivel = 0.
                    height = 0.
                    now = self.get_clock().now()
                    joint_state.header.stamp = now.to_msg()
                    joint_state.name = ['metacarpals_joint', 'metacarpals_2_joint', 'proximal_joint', 'intermediate_joint']
                    

                    a = [hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x,
                         hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].y,
                         hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].z]

                    b = [hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC].x,
                         hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC].y,
                         hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC].z]

                    c = [hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP].x,
                         hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP].y,
                         hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP].z]
                     
                    wrist_thumb_cmc_angle = self.angle_2p_3d(a, b, c)
                    joint_state.position = [wrist_thumb_cmc_angle, wrist_thumb_cmc_angle, wrist_thumb_cmc_angle, wrist_thumb_cmc_angle]
                    
                    print(
                            f'Thumb wrist  angle: (',
                            f'{wrist_thumb_cmc_angle},)'
                        )

                    #draw landmarsk for hand_image topic
                    self.mp_drawing.draw_landmarks(
                        image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

 
            self.joint_pub.publish(joint_state) 

            self.hand_image_publisher_.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
            # self.hand_msg_publisher_.publish(msg)



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