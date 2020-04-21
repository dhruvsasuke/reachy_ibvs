import numpy as np
import cv2


red_lower = np.array ([0, 50 , 50], np.uint8)
red_upper = np.array ([20, 255, 255], np.uint8)
green_lower = np.array([55, 1, 1], np.uint8)
green_upper = np.array([80, 255, 255], np.uint8)
blue_lower = np.array([100, 150, 50], np.uint8)
blue_upper = np.array([120, 255, 255], np.uint8)
yellow_lower = np.array ([25, 60, 50], np.uint8)
yellow_upper = np.array ([45, 255, 255], np.uint8)
kernal = np.ones((15,15), "uint8")


def extract_features(hsv_image, color="red"):
    global red_lower, red_upper, green_lower, green_upper
    global blue_lower, blue_upper, yellow_lower, yellow_upper
    
    if(color == "red"):
        mask = cv2.inRange(hsv_image, red_lower, red_upper)
    elif (color == "blue"):
        mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
    elif (color == "green"):
        mask = cv2.inRange(hsv_image, green_lower, green_upper)
    elif (color == "yellow"):
        mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
    else:
        return (-1,-1)
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)
    (_, cnts, _) =cv2.findContours (mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-3:]
    sorted_cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
    cnt = sorted_cnts[0]
    moments = cv2.moments(cnt)
    centre = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
    return centre