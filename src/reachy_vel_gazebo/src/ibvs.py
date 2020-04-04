#!/usr/bin/env python

import rospy
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from PIL import Image as pil_img
from cv_bridge import CvBridge
import cv2
import math

jac_array = np.zeros(42)
jac = np.zeros((6,7))

vel_ee = np.zeros(6)
vel_ee[1] = 0.01

vel_joints = np.zeros(7)
joint_states = np.zeros(7)


centre_red = (0,0)
centre_green = (0,0)
centre_yellow = (0,0)
centre_blue = (0,0)

rgb_img = np.zeros((480,640,3),np.uint8)
hsv_img = np.zeros((480,640,3),np.uint8)

#################################### Upper and Lower limit for every color ################################################

red_lower = np.array ([0, 50 , 50], np.uint8)
red_upper = np.array ([20, 255, 255], np.uint8)
green_lower = np.array([55, 1, 1], np.uint8)
green_upper = np.array([80, 255, 255], np.uint8)
blue_lower = np.array([100, 150, 50], np.uint8)
blue_upper = np.array([120, 255, 255], np.uint8)
yellow_lower = np.array ([25, 60, 50], np.uint8)
yellow_upper = np.array ([45, 255, 255], np.uint8)
kernal = np.ones((15,15), "uint8")


Lc = np.zeros([8,6])
error = np.zeros(8)

curr_features = np.zeros((4,3))
des_features = np.array([[417, 274, 0.527378738], [292, 273, 0.410142422], [287, 149, 1.05019236], [417, 144, 0.536199331]])

#################################### Velocity Publisher for every joint ################################################

pub1 = rospy.Publisher("/reachy/shoulder_pitch_velocity_controller/command", Float64, queue_size=10)
pub2 = rospy.Publisher("/reachy/shoulder_roll_velocity_controller/command", Float64, queue_size=10)
pub3 = rospy.Publisher("/reachy/arm_yaw_velocity_controller/command", Float64, queue_size=10)
pub4 = rospy.Publisher("/reachy/elbow_pitch_velocity_controller/command", Float64, queue_size=10)
pub5 = rospy.Publisher("/reachy/forearm_yaw_velocity_controller/command", Float64, queue_size=10)
pub6 = rospy.Publisher("/reachy/wrist_pitch_velocity_controller/command", Float64, queue_size=10)
pub7 = rospy.Publisher("/reachy/wrist_roll_velocity_controller/command", Float64, queue_size=10)



def Jacobian_Callback(data):
    global jac_array, jac, joint_states, vel_ee, vel_joints
    global pub1, pub2, pub3, pub4, pub5, pub6, pub7
    jac_array = np.asarray(data.data)
    jac = jac_array.reshape((6,7))
    jac_inverse = np.linalg.pinv(jac)
    vel_joints = np.matmul(jac_inverse, vel_ee)
    print(jac_inverse, vel_joints)
    # print(vel_joints)

    pub1.publish(vel_joints[0])
    pub2.publish(vel_joints[1])
    pub3.publish(vel_joints[2])
    pub4.publish(vel_joints[3])
    pub5.publish(vel_joints[4])
    pub6.publish(vel_joints[5])
    pub7.publish(vel_joints[6])         
    

def Joint_State_Callback(data):
    global joint_states
    joint_states = np.asarray(data.position)
    # print(joint_states)



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
    global Lc, vel_ee, error
    update_interaction_matrix()
    error = np.reshape((curr_features[:,:-1]-des_features[:,:-1]), (8,1))
    # print(math.sqrt(np.sum(np.square(error))))
    L_inverse = np.linalg.pinv(Lc)
    L_transpose = np.transpose(Lc)
    K = 0.2
    # print(shape(L_inverse))
    # print(shape(error))
    vel_ee = -K * np.matmul(L_inverse, error)
    # print(vel_ee)


def Img_RGB_Callback(rgb_data):
    # print("rgb")
    global kernal, rgb_img, hsv_img, centre_red, centre_blue, centre_green, centre_yellow
    global des_features, curr_features, Lc
    global joint_states, vel_ee, vel_joints
    global error
    bridge = CvBridge()
    rgb_img = bridge.imgmsg_to_cv2(rgb_data, desired_encoding='bgr8')
    hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)

    red_mask = cv2.inRange(hsv_img, red_lower, red_upper)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernal)
    (_, cnts_red, _) =cv2.findContours (red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-3:]
    sorted_cnts_red = sorted(cnts_red, key=cv2.contourArea, reverse=True)
    cnt_red = sorted_cnts_red[0]
    moments_red = cv2.moments(cnt_red)
    centre_red = (int(moments_red['m10'] / moments_red['m00']), int(moments_red['m01'] / moments_red['m00']))
    
    curr_features[0][0] = centre_red[0]
    curr_features[0][1] = centre_red[1]

    blue_mask = cv2.inRange(hsv_img, blue_lower, blue_upper)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernal)
    (_, cnts_blue, _) =cv2.findContours (blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-3:]
    sorted_cnts_blue = sorted(cnts_blue, key=cv2.contourArea, reverse=True)
    cnt_blue = sorted_cnts_blue[0]
    moments_blue = cv2.moments(cnt_blue)
    centre_blue = (int(moments_blue['m10'] / moments_blue['m00']), int(moments_blue['m01'] / moments_blue['m00']))
    
    curr_features[1][0] = centre_blue[0]
    curr_features[1][1] = centre_blue[1]

    green_mask = cv2.inRange(hsv_img, green_lower, green_upper)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernal)
    (_, cnts_green, _) =cv2.findContours (green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-3:]
    sorted_cnts_green = sorted(cnts_green, key=cv2.contourArea, reverse=True)
    cnt_green = sorted_cnts_green[0]
    moments_green = cv2.moments(cnt_green)
    centre_green = (int(moments_green['m10'] / moments_green['m00']), int(moments_green['m01'] / moments_green['m00']))
    
    curr_features[2][0] = centre_green[0]
    curr_features[2][1] = centre_green[1]    

    yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernal)
    (_, cnts_yellow, _) =cv2.findContours (yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-3:]
    sorted_cnts_yellow = sorted(cnts_yellow, key=cv2.contourArea, reverse=True)
    cnt_yellow = sorted_cnts_yellow[0]
    moments_yellow = cv2.moments(cnt_yellow)
    centre_yellow = (int(moments_yellow['m10'] / moments_yellow['m00']), int(moments_yellow['m01'] / moments_yellow['m00']))
    
    curr_features[3][0] = centre_yellow[0]
    curr_features[3][1] = centre_yellow[1]    

    cv2.circle(rgb_img, (int(des_features[0][0]),int(des_features[0][1])), 5, (255, 0, 255), -1)
    cv2.circle(rgb_img, (int(des_features[1][0]),int(des_features[1][1])), 5, (255, 0, 255), -1)
    cv2.circle(rgb_img, (int(des_features[2][0]),int(des_features[2][1])), 5, (255, 0, 255), -1)
    cv2.circle(rgb_img, (int(des_features[3][0]),int(des_features[3][1])), 5, (255, 0, 255), -1)
    cv2.imshow("sample", rgb_img)

    # print(curr_features)
    print(curr_features, math.sqrt(np.sum(np.square(error))))

    update_joint_velocity()


    cv2.waitKey(1)

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
    rospy.Subscriber("/jacobian_topic", Float64MultiArray, Jacobian_Callback)

def get_image():
    rospy.Subscriber("/camera/rgb/image_raw", Image, Img_RGB_Callback)
    rospy.Subscriber("/camera/depth/image_raw", Image, Image_Depth_Callback)

def main():
    global jac
    get_jacobian()
    get_image()
    rospy.spin()


if __name__=='__main__':
    main()