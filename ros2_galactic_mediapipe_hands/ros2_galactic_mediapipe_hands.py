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

from tf2_ros import TransformBroadcaster , TransformStamped
# from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from math import sin, cos, pi
from rclpy.qos import QoSProfile

from ros2_galactic_mediapipe_hands_interfaces.msg import Hand

import numpy as np


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.hand_image_publisher_ = self.create_publisher(sensor_msgs.msg.Image, 'hand_image', 10)
        self.hand_msg_publisher_ = self.create_publisher(Hand, 'hand_msg', 10)
        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

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
            odom_trans.child_frame_id = 'metacarpals1'

            odom_trans0 = TransformStamped()
            odom_trans0.header.frame_id = 'wrist'
            odom_trans0.child_frame_id = 'index_finger_mcp'

            odom_trans1 = TransformStamped()
            odom_trans1.header.frame_id = 'index_finger_mcp'
            odom_trans1.child_frame_id = 'index_finger_pip'

            odom_trans2 = TransformStamped()
            odom_trans2.header.frame_id = 'index_finger_pip'
            odom_trans2.child_frame_id = 'index_finger_dip'

            odom_trans3 = TransformStamped()
            odom_trans3.header.frame_id = 'index_finger_dip'
            odom_trans3.child_frame_id = 'index_finger_tip'

            odom_trans4 = TransformStamped()
            odom_trans4.header.frame_id = 'wrist'
            odom_trans4.child_frame_id = 'thumb_cmc'

            odom_trans5 = TransformStamped()
            odom_trans5.header.frame_id = 'thumb_cmc'
            odom_trans5.child_frame_id = 'thumb_mcp'

            odom_trans6 = TransformStamped()
            odom_trans6.header.frame_id = 'thumb_mcp'
            odom_trans6.child_frame_id = 'thumb_ip'

            odom_trans7 = TransformStamped()
            odom_trans7.header.frame_id = 'thumb_ip'
            odom_trans7.child_frame_id = 'thumb_tip'

            odom_trans8 = TransformStamped()
            odom_trans8.header.frame_id = 'index_finger_mcp'
            odom_trans8.child_frame_id = 'middle_finger_mcp'

            odom_trans9 = TransformStamped()
            odom_trans9.header.frame_id = 'middle_finger_mcp'
            odom_trans9.child_frame_id = 'middle_finger_pip'

            odom_trans10 = TransformStamped()
            odom_trans10.header.frame_id = 'middle_finger_pip'
            odom_trans10.child_frame_id = 'middle_finger_dip'

            odom_trans11 = TransformStamped()
            odom_trans11.header.frame_id = 'middle_finger_dip'
            odom_trans11.child_frame_id = 'middle_finger_tip'

            odom_trans12 = TransformStamped()
            odom_trans12.header.frame_id = 'middle_finger_mcp'
            odom_trans12.child_frame_id = 'ring_finger_mcp'

            odom_trans13 = TransformStamped()
            odom_trans13.header.frame_id = 'ring_finger_mcp'
            odom_trans13.child_frame_id = 'ring_finger_pip'

            odom_trans14 = TransformStamped()
            odom_trans14.header.frame_id = 'ring_finger_pip'
            odom_trans14.child_frame_id = 'ring_finger_dip'

            odom_trans15 = TransformStamped()
            odom_trans15.header.frame_id = 'ring_finger_dip'
            odom_trans15.child_frame_id = 'ring_finger_tip'

            odom_trans16 = TransformStamped()
            odom_trans16.header.frame_id = 'ring_finger_mcp'
            odom_trans16.child_frame_id = 'pinky_mcp'

            odom_trans17 = TransformStamped()
            odom_trans17.header.frame_id = 'pinky_mcp'
            odom_trans17.child_frame_id = 'pinky_pip'

            odom_trans18 = TransformStamped()
            odom_trans18.header.frame_id = 'pinky_pip'
            odom_trans18.child_frame_id = 'pinky_dip'

            odom_trans19 = TransformStamped()
            odom_trans19.header.frame_id = 'pinky_dip'
            odom_trans19.child_frame_id = 'pinky_tip'




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
                    odom_trans.transform.translation.x = hand_landmarks.landmark[0].x * iwidth
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

                    #new
                    odom_trans4.header.stamp = now.to_msg()
                    odom_trans4.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC].x * iwidth
                    odom_trans4.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC].y * iheight
                    odom_trans4.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC].z
                    odom_trans4.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans5.header.stamp = now.to_msg()
                    odom_trans5.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP].x * iwidth
                    odom_trans5.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP].y * iheight
                    odom_trans5.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_MCP].z
                    odom_trans5.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans6.header.stamp = now.to_msg()
                    odom_trans6.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP].x * iwidth
                    odom_trans6.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP].y * iheight
                    odom_trans6.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP].z
                    odom_trans6.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans7.header.stamp = now.to_msg()
                    odom_trans7.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].x * iwidth
                    odom_trans7.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].y * iheight
                    odom_trans7.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP].z
                    odom_trans7.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans8.header.stamp = now.to_msg()
                    odom_trans8.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x * iwidth
                    odom_trans8.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y * iheight
                    odom_trans8.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP].z
                    odom_trans8.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans9.header.stamp = now.to_msg()
                    odom_trans9.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP].x * iwidth
                    odom_trans9.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y * iheight
                    odom_trans9.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP].z
                    odom_trans9.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans10.header.stamp = now.to_msg()
                    odom_trans10.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_DIP].x * iwidth
                    odom_trans10.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_DIP].y * iheight
                    odom_trans10.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_DIP].z
                    odom_trans10.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans11.header.stamp = now.to_msg()
                    odom_trans11.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x * iwidth
                    odom_trans11.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y * iheight
                    odom_trans11.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].z
                    odom_trans11.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans12.header.stamp = now.to_msg()
                    odom_trans12.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP].x * iwidth
                    odom_trans12.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP].y * iheight
                    odom_trans12.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_MCP].z
                    odom_trans12.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans13.header.stamp = now.to_msg()
                    odom_trans13.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP].x * iwidth
                    odom_trans13.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP].y * iheight
                    odom_trans13.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP].z
                    odom_trans13.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans14.header.stamp = now.to_msg()
                    odom_trans14.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_DIP].x * iwidth
                    odom_trans14.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_DIP].y * iheight
                    odom_trans14.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_DIP].z
                    odom_trans14.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans15.header.stamp = now.to_msg()
                    odom_trans15.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].x * iwidth
                    odom_trans15.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].y * iheight
                    odom_trans15.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP].z
                    odom_trans15.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans16.header.stamp = now.to_msg()
                    odom_trans16.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].x * iwidth
                    odom_trans16.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].y * iheight
                    odom_trans16.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_MCP].z
                    odom_trans16.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans17.header.stamp = now.to_msg()
                    odom_trans17.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP].x * iwidth
                    odom_trans17.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP].y * iheight
                    odom_trans17.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP].z
                    odom_trans17.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans18.header.stamp = now.to_msg()
                    odom_trans18.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_DIP].x * iwidth
                    odom_trans18.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_DIP].y * iheight
                    odom_trans18.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_DIP].z
                    odom_trans18.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    odom_trans19.header.stamp = now.to_msg()
                    odom_trans19.transform.translation.x = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].x * iwidth
                    odom_trans19.transform.translation.y = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].y * iheight
                    odom_trans19.transform.translation.z = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP].z
                    odom_trans19.transform.rotation = self.euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                    #draw landmarsk for hand_image topic
                    self.mp_drawing.draw_landmarks(
                        image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)


            self.broadcaster.sendTransform(odom_trans)
            self.broadcaster.sendTransform(odom_trans0)
            self.broadcaster.sendTransform(odom_trans1)
            self.broadcaster.sendTransform(odom_trans2)
            self.broadcaster.sendTransform(odom_trans3)
            self.broadcaster.sendTransform(odom_trans4)
            self.broadcaster.sendTransform(odom_trans5)
            self.broadcaster.sendTransform(odom_trans6)
            self.broadcaster.sendTransform(odom_trans7)
            self.broadcaster.sendTransform(odom_trans8)
            self.broadcaster.sendTransform(odom_trans9)
            self.broadcaster.sendTransform(odom_trans10)
            self.broadcaster.sendTransform(odom_trans11)
            self.broadcaster.sendTransform(odom_trans12)
            self.broadcaster.sendTransform(odom_trans13)
            self.broadcaster.sendTransform(odom_trans14)
            self.broadcaster.sendTransform(odom_trans15)
            self.broadcaster.sendTransform(odom_trans16)
            self.broadcaster.sendTransform(odom_trans17)
            self.broadcaster.sendTransform(odom_trans18)
            self.broadcaster.sendTransform(odom_trans19)



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