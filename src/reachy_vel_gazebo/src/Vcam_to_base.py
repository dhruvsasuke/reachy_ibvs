import numpy as np
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


d = np.array([l0+l1,0,-(l2+l3),0,-(l4+l5),0])
a = np.array([0,0,0,0,0,0])
alp = np.array([m.pi/2, m.pi/2, m.pi/2, m.pi/2, -m.pi/2, m.pi/2])

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