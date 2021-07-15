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

from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Quaternion
from math import sin, cos, pi
from rclpy.qos import QoSProfile

from ros2_galactic_mediapipe_hands_interfaces.msg import Hand


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.hand_image_publisher_ = self.create_publisher(sensor_msgs.msg.Image, 'hand_image', 10)
        self.hand_msg_publisher_ = self.create_publisher(Hand, 'hand_msg', 10)
        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
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
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
        
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
            
            odom_trans = TransformStamped()
            odom_trans.header.frame_id = 'odom'
            odom_trans.child_frame_id = 'wrist1'
            
            odom_trans0 = TransformStamped()
            odom_trans0.header.frame_id = 'wrist1'
            odom_trans0.child_frame_id = 'metacarpals1'
            
            odom_trans1 = TransformStamped()
            odom_trans1.header.frame_id = 'metacarpals1'
            odom_trans1.child_frame_id = 'proximal1'
            
            odom_trans2 = TransformStamped()
            odom_trans2.header.frame_id = 'proximal1'
            odom_trans2.child_frame_id = 'intermediate1'
            
            odom_trans3 = TransformStamped()
            odom_trans3.header.frame_id = 'intermediate1'
            odom_trans3.child_frame_id = 'distal1'
                    
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
                    
                    
                    now = self.get_clock().now()
                    odom_trans.header.stamp = now.to_msg()
                    odom_trans.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x * iwidth
                    odom_trans.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].y * iheight
                    odom_trans.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].z
                    odom_trans.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw
                    
                    odom_trans0.header.stamp = now.to_msg()
                    odom_trans0.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].x * iwidth
                    odom_trans0.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y * iheight
                    odom_trans0.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].z
                    odom_trans0.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw
                    
                    odom_trans1.header.stamp = now.to_msg()
                    odom_trans1.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].x * iwidth
                    odom_trans1.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].y * iheight
                    odom_trans1.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP].z
                    odom_trans1.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw
                    
                    odom_trans2.header.stamp = now.to_msg()
                    odom_trans2.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_DIP].x * iwidth
                    odom_trans2.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_DIP].y * iheight
                    odom_trans2.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_DIP].z
                    odom_trans2.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw
                    
                    odom_trans3.header.stamp = now.to_msg()
                    odom_trans3.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x * iwidth
                    odom_trans3.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y * iheight
                    odom_trans3.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].z
                    odom_trans3.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw
                    # odom_trans.transform.rotation = \
                        # euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw
                    

                    
                    # self.mp_drawing.draw_landmarks(
                    #     image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            
            self.broadcaster.sendTransform(odom_trans)
            self.broadcaster.sendTransform(odom_trans0)
            self.broadcaster.sendTransform(odom_trans1)
            self.broadcaster.sendTransform(odom_trans2)
            self.broadcaster.sendTransform(odom_trans3)

            self.hand_image_publisher_.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
            self.hand_msg_publisher_.publish(msg)
   
            

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