#!/usr/bin/env python
#To calculate jacobian
import numpy as numpy
def Jacobians_base_hand_ball(q) :   
    shoulder_pitch=q[1]
    shoulder_roll=q[2]
    arm_yaw=q[3]
    elbow_pitch=q[4]
    forearm_yaw=q[5]
    wrist_pitch=q[6]
    wrist_roll=q[7]
    J_base_shoulder= np.zeros(6,1)
    J_base_shoulder[0,0]=0
    J_base_shoulder[1,0]=0
    J_base_shoulder[2,0]=0
    J_base_shoulder[3,0]=0
    J_base_shoulder[4,0]=1.00000000000000
    J_base_shoulder[5,0]=0

    J_base_shoulder_to_arm=np.zeros(6,2)
    J_base_shoulder_to_arm[0,0]=0
    J_base_shoulder_to_arm[0,1]=0
    J_base_shoulder_to_arm[1,0]=0
    J_base_shoulder_to_arm[1,1]=0
    J_base_shoulder_to_arm[2,0]=0
    J_base_shoulder_to_arm[2,1]=0
    J_base_shoulder_to_arm[3,0]=0
    J_base_shoulder_to_arm[3,1]=1.0*cos(shoulder_pitch)
    J_base_shoulder_to_arm[4,0]=1.00000000000000
    J_base_shoulder_to_arm[4,1]=0
    J_base_shoulder_to_arm[5,0]=0
    J_base_shoulder_to_arm[5,1]=-1.0*sin(shoulder_pitch)


    J_base_upper_arm=np.zeros(6,3)
    J_base_upper_arm[0,0]=0
    J_base_upper_arm[0,1]=0
    J_base_upper_arm[0,2]=0
    J_base_upper_arm[1,0]=0
    J_base_upper_arm[1,1]=0
    J_base_upper_arm[1,2]=0
    J_base_upper_arm[2,0]=0
    J_base_upper_arm[2,1]=0
    J_base_upper_arm[2,2]=0
    J_base_upper_arm[3,0]=0
    J_base_upper_arm[3,1]=1.0*cos(shoulder_pitch)
    J_base_upper_arm[3,2]=1.0*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_upper_arm[4,0]=1.00000000000000
    J_base_upper_arm[4,1]=0
    J_base_upper_arm[4,2]=-1.0*sin(shoulder_roll)
    J_base_upper_arm[5,0]=0
    J_base_upper_arm[5,1]=-1.0*sin(shoulder_pitch)
    J_base_upper_arm[5,2]=1.0*cos(shoulder_pitch)*cos(shoulder_roll)




    J_base_forearm =np.zeros(6,4)
    
    J_base_forearm[0,0]=-0.30745*cos(shoulder_pitch)*cos(shoulder_roll)
    J_base_forearm[0,1]=0.30745*sin(shoulder_pitch)*sin(shoulder_roll)
    J_base_forearm[0,2]=0
    J_base_forearm[0,3]=0
    J_base_forearm[1,0]=0
    J_base_forearm[1,1]=0.30745*sin(shoulder_pitch)**2*cos(shoulder_roll) + 0.30745*cos(shoulder_pitch)**2*cos(shoulder_roll)
    J_base_forearm[1,2]=0
    J_base_forearm[1,3]=0
    J_base_forearm[2,0]=0.30745*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_forearm[2,1]=0.30745*sin(shoulder_roll)*cos(shoulder_pitch)
    J_base_forearm[2,2]=0
    J_base_forearm[2,3]=0
    J_base_forearm[3,0]=0
    J_base_forearm[3,1]=1.0*cos(shoulder_pitch)
    J_base_forearm[3,2]=1.0*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_forearm[3,3]=-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw)
    J_base_forearm[4,0]=1.00000000000000
    J_base_forearm[4,1]=0
    J_base_forearm[4,2]=-1.0*sin(shoulder_roll)
    J_base_forearm[4,3]=1.0*cos(arm_yaw)*cos(shoulder_roll)
    J_base_forearm[5,0]=0
    J_base_forearm[5,1]=-1.0*sin(shoulder_pitch)
    J_base_forearm[5,2]=1.0*cos(shoulder_pitch)*cos(shoulder_roll)
    J_base_forearm[5,3]=1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch)
    J_base_forearm[0,0]=-0.30745*cos(shoulder_pitch)*cos(shoulder_roll)
    J_base_forearm[0,1]=0.30745*sin(shoulder_pitch)*sin(shoulder_roll)
    J_base_forearm[0,2]=0
    J_base_forearm[0,3]=0
    J_base_forearm[1,0]=0
    J_base_forearm[1,1]=0.30745*sin(shoulder_pitch)**2*cos(shoulder_roll) + 0.30745*cos(shoulder_pitch)**2*cos(shoulder_roll)
    J_base_forearm[1,2]=0
    J_base_forearm[1,3]=0
    J_base_forearm[2,0]=0.30745*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_forearm[2,1]=0.30745*sin(shoulder_roll)*cos(shoulder_pitch)
    J_base_forearm[2,2]=0
    J_base_forearm[2,3]=0
    J_base_forearm[3,0]=0
    J_base_forearm[3,1]=1.0*cos(shoulder_pitch)
    J_base_forearm[3,2]=1.0*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_forearm[3,3]=-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw)
    J_base_forearm[4,0]=1.00000000000000
    J_base_forearm[4,1]=0
    J_base_forearm[4,2]=-1.0*sin(shoulder_roll)
    J_base_forearm[4,3]=1.0*cos(arm_yaw)*cos(shoulder_roll)
    J_base_forearm[5,0]=0
    J_base_forearm[5,1]=-1.0*sin(shoulder_pitch)
    J_base_forearm[5,2]=1.0*cos(shoulder_pitch)*cos(shoulder_roll)
    J_base_forearm[5,3]=1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch)



    J_base_wrist =np.zeros(6,6)

    J_base_wrist[0,0]=-0.30745*cos(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist[0,1]=0.30745*sin(shoulder_pitch)*sin(shoulder_roll)
    J_base_wrist[0,2]=0
    J_base_wrist[0,3]=0
    J_base_wrist[0,5]=0
    J_base_wrist[1,0]=0
    J_base_wrist[1,1]=0.30745*sin(shoulder_pitch)**2*cos(shoulder_roll) + 0.30745*cos(shoulder_pitch)**2*cos(shoulder_roll)
    J_base_wrist[1,2]=0
    J_base_wrist[1,3]=0
    J_base_wrist[1,5]=0
    J_base_wrist[2,0]=0.30745*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist[2,1]=0.30745*sin(shoulder_roll)*cos(shoulder_pitch)
    J_base_wrist[2,2]=0
    J_base_wrist[2,3]=0
    J_base_wrist[2,5]=0
    J_base_wrist[3,0]=0
    J_base_wrist[3,1]=1.0*cos(shoulder_pitch)
    J_base_wrist[3,2]=1.0*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist[3,3]=-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw)
    J_base_wrist[3,5]=1.0*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll)
    J_base_wrist[4,0]=1.00000000000000
    J_base_wrist[4,1]=0
    J_base_wrist[4,2]=-1.0*sin(shoulder_roll)
    J_base_wrist[4,3]=1.0*cos(arm_yaw)*cos(shoulder_roll)
    J_base_wrist[4,5]=1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch)
    J_base_wrist[5,0]=0
    J_base_wrist[5,1]=-1.0*sin(shoulder_pitch)
    J_base_wrist[5,2]=1.0*cos(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist[5,3]=1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch)
    J_base_wrist[5,5]=1.0*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll)


    J_base_wrist_hand =np.zeros(6,7)
    J_base_wrist_hand[0,1]=-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist_hand[0,2]=1.0*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll))*sin(shoulder_pitch)
    J_base_wrist_hand[0,3]=-1.0*(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll))*sin(shoulder_roll) - 1.0*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll))*cos(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist_hand[0,4]=1.0*(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(arm_yaw)*cos(shoulder_roll) + (-1.0*sin(arm_yaw)*sin(shoulder_pitch) - 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch))
    J_base_wrist_hand[0,5]=(-1.0*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch)) + (-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*(0.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch))
    J_base_wrist_hand[0,6]=0
    J_base_wrist_hand[1,1]=0
    J_base_wrist_hand[1,2]=-1.0*(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) - 0.30745*sin(shoulder_pitch)*cos(shoulder_roll))*sin(shoulder_pitch) - 1.0*(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll))*cos(shoulder_pitch)
    J_base_wrist_hand[1,3]=1.0*(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) - 0.30745*sin(shoulder_pitch)*cos(shoulder_roll))*cos(shoulder_pitch)*cos(shoulder_roll) - 1.0*(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll))*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist_hand[1,4]=(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*(0.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch)) + (-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*(0.0*sin(arm_yaw)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))
    J_base_wrist_hand[1,5]=(-1.0*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll)) + (-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*(0.0*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))
    J_base_wrist_hand[1,6]=0
    J_base_wrist_hand[2,1]=0.22425*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) + 0.30745*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist_hand[2,2]=1.0*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll))*cos(shoulder_pitch)
    J_base_wrist_hand[2,3]=1.0*(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) - 0.30745*sin(shoulder_pitch)*cos(shoulder_roll))*sin(shoulder_roll) + 1.0*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll))*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist_hand[2,4]=-1.0*(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*cos(arm_yaw)*cos(shoulder_roll) + (-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch))
    J_base_wrist_hand[2,5]=(-0.22425*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*(-1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(shoulder_roll)*cos(elbow_pitch)) + (0.0*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch))
    J_base_wrist_hand[2,6]=0
    J_base_wrist_hand[3,1]=0
    J_base_wrist_hand[3,2]=1.0*cos(shoulder_pitch)
    J_base_wrist_hand[3,3]=1.0*sin(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist_hand[3,4]=-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw)
    J_base_wrist_hand[3,5]=1.0*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll)
    J_base_wrist_hand[3,6]=-1.0*(0.0*(0.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*cos(forearm_yaw)
    J_base_wrist_hand[4,1]=1.00000000000000
    J_base_wrist_hand[4,2]=0
    J_base_wrist_hand[4,3]=-1.0*sin(shoulder_roll)
    J_base_wrist_hand[4,4]=1.0*cos(arm_yaw)*cos(shoulder_roll)
    J_base_wrist_hand[4,5]=1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch)
    J_base_wrist_hand[4,6]=-1.0*(0.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*sin(forearm_yaw) + 1.0*cos(arm_yaw)*cos(forearm_yaw)*cos(shoulder_roll)
    J_base_wrist_hand[5,1]=0
    J_base_wrist_hand[5,2]=-1.0*sin(shoulder_pitch)
    J_base_wrist_hand[5,3]=1.0*cos(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist_hand[5,4]=1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch)
    J_base_wrist_hand[5,5]=1.0*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll)
    J_base_wrist_hand[5,6]=-1.0*(0.0*(0.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(0.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*cos(forearm_yaw)



    J_base_hand_ball =np.zeros(6,8)
    
    J_base_hand_ball[1,1]=-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] - 0.22425*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll] - 0.30745*cos[shoulder_pitch]*cos[shoulder_roll]
    J_base_hand_ball[1,2]=1.0*[-0.03243*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*sin[wrist_pitch] - 0.03243*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*cos[wrist_pitch] - 0.22425*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] + 0.22425*sin[shoulder_roll]*cos[elbow_pitch] + 0.30745*sin[shoulder_roll]]*sin[shoulder_pitch]
    J_base_hand_ball[1,3]=-1.0*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] - 0.22425*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll] - 0.30745*cos[shoulder_pitch]*cos[shoulder_roll]]*sin[shoulder_roll] - 1.0*[-0.03243*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*sin[wrist_pitch] - 0.03243*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*cos[wrist_pitch] - 0.22425*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] + 0.22425*sin[shoulder_roll]*cos[elbow_pitch] + 0.30745*sin[shoulder_roll]]*cos[shoulder_pitch]*cos[shoulder_roll]
    J_base_hand_ball[1,4]=[-1.0*sin[arm_yaw]*sin[shoulder_pitch] - 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*[-0.03243*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*sin[wrist_pitch] - 0.03243*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*cos[wrist_pitch] - 0.22425*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] + 0.22425*sin[shoulder_roll]*cos[elbow_pitch]] + 1.0*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] - 0.22425*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[arm_yaw]*cos[shoulder_roll]
    J_base_hand_ball[1,5]=[-1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] - 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*[-0.03243*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*sin[wrist_pitch] - 0.03243*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*cos[wrist_pitch] - 0.22425*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] + 0.22425*sin[shoulder_roll]*cos[elbow_pitch]] + [1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] - 0.22425*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]
    J_base_hand_ball[1,6]=[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[wrist_pitch]]*[-1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*sin[forearm_yaw] + 1.0*cos[arm_yaw]*cos[forearm_yaw]*cos[shoulder_roll]] + [1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*sin[forearm_yaw] - 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*cos[forearm_yaw]]*[-0.03243*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*sin[wrist_pitch] - 0.03243*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*cos[wrist_pitch]]
    J_base_hand_ball[1,7]=0
    J_base_hand_ball[2,1]=0
    J_base_hand_ball[2,2]=-1.0*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] - 0.22425*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll] - 0.30745*sin[shoulder_pitch]*cos[shoulder_roll]]*sin[shoulder_pitch] - 1.0*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] - 0.22425*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll] - 0.30745*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[shoulder_pitch]
    J_base_hand_ball[2,3]=1.0*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] - 0.22425*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll] - 0.30745*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[shoulder_pitch]*cos[shoulder_roll] - 1.0*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] - 0.22425*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll] - 0.30745*cos[shoulder_pitch]*cos[shoulder_roll]]*sin[shoulder_pitch]*cos[shoulder_roll]
    J_base_hand_ball[2,4]=[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] - 0.22425*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]] + [1.0*sin[arm_yaw]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] - 0.22425*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]
    J_base_hand_ball[2,5]=[-1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] - 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] - 0.22425*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]] + [1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] - 0.22425*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]
    J_base_hand_ball[2,6]=[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[wrist_pitch]]*[-1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*sin[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*cos[forearm_yaw]] + [-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[wrist_pitch]]*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*sin[forearm_yaw] - 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*cos[forearm_yaw]]
    J_base_hand_ball[2,7]=0
    J_base_hand_ball[3,1]=0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*sin[wrist_pitch] + 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] + 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 0.22425*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll] + 0.30745*sin[shoulder_pitch]*cos[shoulder_roll]
    J_base_hand_ball[3,2]=1.0*[-0.03243*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*sin[wrist_pitch] - 0.03243*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*cos[wrist_pitch] - 0.22425*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] + 0.22425*sin[shoulder_roll]*cos[elbow_pitch] + 0.30745*sin[shoulder_roll]]*cos[shoulder_pitch]
    J_base_hand_ball[3,3]=1.0*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] - 0.22425*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll] - 0.30745*sin[shoulder_pitch]*cos[shoulder_roll]]*sin[shoulder_roll] + 1.0*[-0.03243*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*sin[wrist_pitch] - 0.03243*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*cos[wrist_pitch] - 0.22425*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] + 0.22425*sin[shoulder_roll]*cos[elbow_pitch] + 0.30745*sin[shoulder_roll]]*sin[shoulder_pitch]*cos[shoulder_roll]
    J_base_hand_ball[3,4]=[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*[-0.03243*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*sin[wrist_pitch] - 0.03243*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*cos[wrist_pitch] - 0.22425*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] + 0.22425*sin[shoulder_roll]*cos[elbow_pitch]] - 1.0*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] - 0.22425*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[arm_yaw]*cos[shoulder_roll]
    J_base_hand_ball[3,5]=[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*[-0.03243*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*sin[wrist_pitch] - 0.03243*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*cos[wrist_pitch] - 0.22425*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] + 0.22425*sin[shoulder_roll]*cos[elbow_pitch]] + [-1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[wrist_pitch] - 0.22425*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] - 0.22425*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]
    J_base_hand_ball[3,6]=[-0.03243*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*sin[wrist_pitch] - 0.03243*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*cos[wrist_pitch]]*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*sin[forearm_yaw] - 1.0*cos[arm_yaw]*cos[forearm_yaw]*cos[shoulder_roll]] + [-1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*sin[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*cos[forearm_yaw]]*[-0.03243*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*sin[wrist_pitch] - 0.03243*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*cos[wrist_pitch]]
    J_base_hand_ball[3,7]=0
    J_base_hand_ball[4,1]=0
    J_base_hand_ball[4,2]=1.0*cos[shoulder_pitch]
    J_base_hand_ball[4,3]=1.0*sin[shoulder_pitch]*cos[shoulder_roll]
    J_base_hand_ball[4,4]=-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]
    J_base_hand_ball[4,5]=1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]
    J_base_hand_ball[4,6]=-1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*sin[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*cos[forearm_yaw]
    J_base_hand_ball[4,7]=1.0*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*sin[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[-1.0*sin[arm_yaw]*cos[shoulder_pitch] + 1.0*sin[shoulder_pitch]*sin[shoulder_roll]*cos[arm_yaw]]*sin[forearm_yaw]]*cos[wrist_pitch] - 1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch]*sin[shoulder_roll] + 1.0*cos[arm_yaw]*cos[shoulder_pitch]]*sin[elbow_pitch] + 1.0*sin[shoulder_pitch]*cos[elbow_pitch]*cos[shoulder_roll]]*sin[wrist_pitch]
    J_base_hand_ball[5,1]=1.00000000000000
    J_base_hand_ball[5,2]=0
    J_base_hand_ball[5,3]=-1.0*sin[shoulder_roll]
    J_base_hand_ball[5,4]=1.0*cos[arm_yaw]*cos[shoulder_roll]
    J_base_hand_ball[5,5]=1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]
    J_base_hand_ball[5,6]=-1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*sin[forearm_yaw] + 1.0*cos[arm_yaw]*cos[forearm_yaw]*cos[shoulder_roll]
    J_base_hand_ball[5,7]=1.0*[1.0*[1.0*sin[arm_yaw]*cos[elbow_pitch]*cos[shoulder_roll] + 1.0*sin[elbow_pitch]*sin[shoulder_roll]]*cos[forearm_yaw] + 1.0*sin[forearm_yaw]*cos[arm_yaw]*cos[shoulder_roll]]*cos[wrist_pitch] - 1.0*[1.0*sin[arm_yaw]*sin[elbow_pitch]*cos[shoulder_roll] - 1.0*sin[shoulder_roll]*cos[elbow_pitch]]*sin[wrist_pitch]
    J_base_hand_ball[6,1]=0
    J_base_hand_ball[6,2]=-1.0*sin[shoulder_pitch]
    J_base_hand_ball[6,3]=1.0*cos[shoulder_pitch]*cos[shoulder_roll]
    J_base_hand_ball[6,4]=1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]
    J_base_hand_ball[6,5]=1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]
    J_base_hand_ball[6,6]=-1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*sin[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*cos[forearm_yaw]
    J_base_hand_ball[6,7]=1.0*[1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*cos[elbow_pitch] - 1.0*sin[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*cos[forearm_yaw] + 1.0*[1.0*sin[arm_yaw]*sin[shoulder_pitch] + 1.0*sin[shoulder_roll]*cos[arm_yaw]*cos[shoulder_pitch]]*sin[forearm_yaw]]*cos[wrist_pitch] - 1.0*[1.0*[1.0*sin[arm_yaw]*sin[shoulder_roll]*cos[shoulder_pitch] - 1.0*sin[shoulder_pitch]*cos[arm_yaw]]*sin[elbow_pitch] + 1.0*cos[elbow_pitch]*cos[shoulder_pitch]*cos[shoulder_roll]]*sin[wrist_pitch]

    J={J_base_shoulder,J_base_shoulder_to_arm,J_base_upper_arm,J_base_forearm,J_base_wrist,J_base_wrist_hand,J_base_hand_ball}
    return J

    ##For testing
def main():
    print(Jacobians_base_hand_ball([1,1,1,1,1,1,0,1]) )


if __name__=='__main__':
    main()