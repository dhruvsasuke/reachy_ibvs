import numpy as np
#from math import sin, cos
import math as m

l0=0
l1=0.03970
l2=0.05200
l3=0.25600
l4=0.09700
l5=0.12700
l6=0.03300

d1 = l0+l1
d2 = 0
d3 = -(l2+l3)
d4 = 0
d5 = -(l4+l5)
d6 = 0
d = np.array([d1,d2,d3,d4,d5,d6])
a = np.array([0,0,0,0,0,0])
alp = np.array([3.142/2, 3.142/2, 3.142/2, 3.142/2, -3.142/2, 3.142/2])

def Vc_2_Base(Vcamera,th):
    i1 = 0
    T = np.identity(4)
    trans = np.identity(4)
    for i1 in range (0,6):
    	T = np.array([[m.cos(th[i1]) , -m.sin(th[i1])*m.cos(alp[i1]) , m.sin(th[i1])*m.sin(alp[i1]) , a[i1]*m.cos(th[i1])] , [m.sin(th[i1]) ,m.cos(th[i1])*m.cos(alp[i1]) , -m.cos(th[i1])*m.sin(alp[i1]) , a[i1]*m.sin(th[i1])] , [0 , m.sin(alp[i1]) , m.cos(alp[i1]) , d[i1]] , [0 , 0 , 0 , 1]])
		
    	trans = np.matmul(trans,T)
		
    Rot = np.array([[trans.item(0,0), trans.item(0,1), trans.item(0,2)], [trans.item(1,0), trans.item(1,1), trans.item(1,2)], [trans.item(2,0), trans.item(2,1), trans.item(2,2)]])
	
    Rot = np.linalg.inv(Rot)
	
    V1 = np.array([Vcamera[0], Vcamera[1], Vcamera[2]])
    V2 = np.array([Vcamera[3], Vcamera[4], Vcamera[5]])
	
    V11 = np.matmul(Rot,V1)
    V22 = np.matmul(Rot,V2)
	
    Vbase = np.array([V11[0], V11[1],  V11[2], V22[0], V22[1], V22[2]]) 

    return Vbase

def getTransformBaseWrist_hand(q, T): 
    shoulder_pitch=q[0]
    shoulder_roll=q[1]
    arm_yaw=q[2]
    elbow_pitch=q[3]
    forearm_yaw=q[4]
    wrist_pitch=q[5]

    T[0][0]=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*sin(forearm_yaw))*cos(wrist_pitch) - 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*sin(wrist_pitch)
    T[0][1]=-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*cos(forearm_yaw)
    T[0][2]=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*sin(forearm_yaw))*sin(wrist_pitch) + 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*cos(wrist_pitch)
    T[0][3]=-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) - 0.30745*sin(shoulder_pitch)*cos(shoulder_roll)
    T[1][0]=1.0*(1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*cos(forearm_yaw) + 1.0*sin(forearm_yaw)*cos(arm_yaw)*cos(shoulder_roll))*cos(wrist_pitch) - 1.0*(1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch))*sin(wrist_pitch)
    T[1][1]=-1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*sin(forearm_yaw) + 1.0*cos(arm_yaw)*cos(forearm_yaw)*cos(shoulder_roll)
    T[1][1]=1.0*(1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*cos(forearm_yaw) + 1.0*sin(forearm_yaw)*cos(arm_yaw)*cos(shoulder_roll))*sin(wrist_pitch) + 1.0*(1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch))*cos(wrist_pitch)
    T[1][3]=-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll) - 0.04
    T[2][0]=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*sin(forearm_yaw))*cos(wrist_pitch) - 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*sin(wrist_pitch)
    T[2][1]=-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*cos(forearm_yaw)
    T[2][2]=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*sin(forearm_yaw))*sin(wrist_pitch) + 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(wrist_pitch)
    T[2][3]=-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll)
    T[3][0]=0
    T[3][1]=0
    T[3][2]=0
    T[3][3]=1.00000000000000