#!/usr/bin/env python

import rospy
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, TransformStamped
from PIL import Image as pil_img
from cv_bridge import CvBridge
import cv2
import math
import tf
import feature_extract
import jacobian_function


jac_inverse = np.zeros((6,6))
trans_matrix = np.zeros((4,4))

vel_ee = np.zeros(6)
vel_cam = np.zeros(6)
vel_ee[0] = -0.01

vel_joints = np.zeros(6)
joint_states = np.zeros(7)


centre_red = (0,0)
centre_green = (0,0)
centre_yellow = (0,0)
centre_blue = (0,0)

rgb_img = np.zeros((480,640,3),np.uint8)
hsv_img = np.zeros((480,640,3),np.uint8)

#################################### Upper and Lower limit for every color ################################################

Lc = np.zeros([8,6])
error = np.zeros(8)

curr_features = np.zeros((4,3))
des_features = np.array([[286, 310, 0.5019744], [186, 294, 0.5689618], [216, 211, 0.55366993], [307, 224, 0.52000195]])

#################################### Velocity Publisher for every joint ################################################

pub1 = rospy.Publisher("/reachy/shoulder_pitch_velocity_controller/command", Float64, queue_size=10)
pub2 = rospy.Publisher("/reachy/shoulder_roll_velocity_controller/command", Float64, queue_size=10)
pub3 = rospy.Publisher("/reachy/arm_yaw_velocity_controller/command", Float64, queue_size=10)
pub4 = rospy.Publisher("/reachy/elbow_pitch_velocity_controller/command", Float64, queue_size=10)
pub5 = rospy.Publisher("/reachy/forearm_yaw_velocity_controller/command", Float64, queue_size=10)
pub6 = rospy.Publisher("/reachy/wrist_pitch_velocity_controller/command", Float64, queue_size=10)
    

def Joint_State_Callback(data):
    global joint_states, jac_inverse
    joint_states = np.asarray(data.position)
    jac_inverse = jacobian_function.calc_jack(joint_states[0], joint_states[1], joint_states[2], joint_states[3], joint_states[4], joint_states[5], joint_states[5])
    print(jac_inverse)


def camera_to_base():
    global trans_matrix, vel_cam, vel_ee
    vel_ee = np.concatenate(((np.matmul(trans_matrix[:-1, :-1], vel_cam[:3])), np.matmul(trans_matrix[:-1, :-1], vel_cam[-3:])), axis=None)
    # print(vel_ee)


def Transform_Callback(data):
    global trans_matrix
    trans = tf.TransformerROS(True, rospy.Duration(10.0))
    trans_matrix = trans.fromTranslationRotation((data.translation.x, data.translation.y, data.translation.z), (data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w))
    # print(trans_matrix)
    camera_to_base()


def update_interaction_matrix():
    global Lc, curr_features
    fl = 530
    j=0
    for i in range(4):
        u = curr_features[i,0]
        v = curr_features[i,1]
        z = curr_features[i,2]
        # z = 1
        Lc[j:j+2,:] = np.array([ [-fl/z, 0, u/z, u*v/fl, -(fl*fl+u*u)/fl, v],
                                 [0, -fl/z, v/z, (fl*fl+v*v)/fl, -u*v/fl, -u]   ])
        j=j+2
    # print (Lc)
    # print ("\n")

def update_joint_velocity():
    global Lc, vel_cam, error
    update_interaction_matrix()
    error = np.reshape((curr_features[:,:-1]-des_features[:,:-1]), (8,1))
    # print(math.sqrt(np.sum(np.square(error))))
    L_inverse = np.linalg.pinv(Lc)
    L_transpose = np.transpose(Lc)
    K = 0.1
    # print(shape(L_inverse))
    # print(shape(error))
    vel_cam = -K * np.matmul(L_inverse, error)
    camera_to_base()
    # print(vel_ee)


def Img_RGB_Callback(rgb_data):
    # print("rgb")
    global rgb_img, hsv_img, centre_red, centre_blue, centre_green, centre_yellow
    global des_features, curr_features, Lc
    global joint_states, vel_ee, vel_joints
    global error
    bridge = CvBridge()
    rgb_img = bridge.imgmsg_to_cv2(rgb_data, desired_encoding='bgr8')
    hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)

#################################### Extracting Features ################################################

    centre_red = feature_extract.extract_features(hsv_img, "red")
    centre_blue = feature_extract.extract_features(hsv_img, "blue")
    centre_green = feature_extract.extract_features(hsv_img, "green")
    centre_yellow = feature_extract.extract_features(hsv_img, "yellow")

    curr_features[0][0] = centre_red[0]
    curr_features[0][1] = centre_red[1]
    curr_features[1][0] = centre_blue[0]
    curr_features[1][1] = centre_blue[1]
    curr_features[2][0] = centre_green[0]
    curr_features[2][1] = centre_green[1]    
    curr_features[3][0] = centre_yellow[0]
    curr_features[3][1] = centre_yellow[1]    

#################################### Extracting Features ################################################

    cv2.circle(rgb_img, (int(des_features[0][0]),int(des_features[0][1])), 5, (255, 0, 255), -1)
    cv2.circle(rgb_img, (int(des_features[1][0]),int(des_features[1][1])), 5, (255, 0, 255), -1)
    cv2.circle(rgb_img, (int(des_features[2][0]),int(des_features[2][1])), 5, (255, 0, 255), -1)
    cv2.circle(rgb_img, (int(des_features[3][0]),int(des_features[3][1])), 5, (255, 0, 255), -1)
    cv2.imshow("sample", rgb_img)

    # print(curr_features, math.sqrt(np.sum(np.square(error))))
    cv2.waitKey(1)
    update_joint_velocity()

def Image_Depth_Callback(depth_data):
    # print("depth")
    global curr_features, centre_yellow, centre_blue, centre_green, centre_red
    bridge = CvBridge()
    depth_img = bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')
    
    cv2.circle(rgb_img, centre_red, 5, (0, 0, 255), -1)
    cv2.circle(rgb_img, centre_blue, 5, (0, 0, 255), -1)
    cv2.circle(rgb_img, centre_green, 5, (0, 0, 255), -1)
    cv2.circle(rgb_img, centre_yellow, 5, (0, 0, 255), -1)

    # if(not math.isnan(depth_img[centre_red[0]][centre_red[1]])):
    curr_features[0][2] = depth_img[centre_red[0]][centre_red[1]]
    # if(not math.isnan(depth_img[centre_blue[0]][centre_blue[1]])):
    curr_features[1][2] = depth_img[centre_blue[0]][centre_blue[1]]
    # if(not math.isnan(depth_img[centre_green[0]][centre_green[1]])):
    curr_features[2][2] = depth_img[centre_green[0]][centre_green[1]]
    # if(not math.isnan(depth_img[centre_yellow[0]][centre_yellow[1]])):
    curr_features[3][2] = depth_img[centre_yellow[0]][centre_yellow[1]]

def get_jacobian():
    rospy.init_node("ibvs", anonymous="True")
    rospy.Subscriber("/reachy/joint_states", JointState, Joint_State_Callback)

def get_transform():
    rospy.Subscriber("/transform_topic", Transform, Transform_Callback)

def get_image():
    rospy.Subscriber("/camera/rgb/image_raw", Image, Img_RGB_Callback)
    rospy.Subscriber("/camera/depth/image_raw", Image, Image_Depth_Callback)

def main():
    global jac_inverse
    get_jacobian()
    # get_image()
    # get_transform()
    rospy.spin()

if __name__=='__main__':
    main()