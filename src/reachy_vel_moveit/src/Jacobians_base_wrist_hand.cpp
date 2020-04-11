void getJacobianBaseShoulder(std::vector<double> q,Eigen::Matrix<double,6,7> &J)
 { 
 J.setZero(); 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];


J(0,0)=0;
J(1,0)=0;
J(2,0)=0;
J(3,0)=0;
J(4,0)=1.00000000000000;
J(5,0)=0;
}


void getJacobianBaseShoulder_to_arm(std::vector<double> q,Eigen::Matrix<double,6,7> &J)
 { 
 J.setZero(); 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];


J(0,0)=0;
J(0,1)=0;
J(1,0)=0;
J(1,1)=0;
J(2,0)=0;
J(2,1)=0;
J(3,0)=0;
J(3,1)=1.0*cos(shoulder_pitch);
J(4,0)=1.00000000000000;
J(4,1)=0;
J(5,0)=0;
J(5,1)=-1.0*sin(shoulder_pitch);
}


void getJacobianBaseUpper_arm(std::vector<double> q,Eigen::Matrix<double,6,7> &J)
 { 
 J.setZero(); 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];


J(0,0)=0;
J(0,1)=0;
J(0,2)=0;
J(1,0)=0;
J(1,1)=0;
J(1,2)=0;
J(2,0)=0;
J(2,1)=0;
J(2,2)=0;
J(3,0)=0;
J(3,1)=1.0*cos(shoulder_pitch);
J(3,2)=1.0*sin(shoulder_pitch)*cos(shoulder_roll);
J(4,0)=1.00000000000000;
J(4,1)=0;
J(4,2)=-1.0*sin(shoulder_roll);
J(5,0)=0;
J(5,1)=-1.0*sin(shoulder_pitch);
J(5,2)=1.0*cos(shoulder_pitch)*cos(shoulder_roll);
}


void getJacobianBaseForearm(std::vector<double> q,Eigen::Matrix<double,6,7> &J)
 { 
 J.setZero(); 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];


J(0,0)=-0.30745*cos(shoulder_pitch)*cos(shoulder_roll);
J(0,1)=0.30745*sin(shoulder_pitch)*sin(shoulder_roll);
J(0,2)=0;
J(0,3)=0;
J(1,0)=0;
J(1,1)=0.30745*sin(shoulder_pitch)**2*cos(shoulder_roll) + 0.30745*cos(shoulder_pitch)**2*cos(shoulder_roll);
J(1,2)=0;
J(1,3)=0;
J(2,0)=0.30745*sin(shoulder_pitch)*cos(shoulder_roll);
J(2,1)=0.30745*sin(shoulder_roll)*cos(shoulder_pitch);
J(2,2)=0;
J(2,3)=0;
J(3,0)=0;
J(3,1)=1.0*cos(shoulder_pitch);
J(3,2)=1.0*sin(shoulder_pitch)*cos(shoulder_roll);
J(3,3)=-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw);
J(4,0)=1.00000000000000;
J(4,1)=0;
J(4,2)=-1.0*sin(shoulder_roll);
J(4,3)=1.0*cos(arm_yaw)*cos(shoulder_roll);
J(5,0)=0;
J(5,1)=-1.0*sin(shoulder_pitch);
J(5,2)=1.0*cos(shoulder_pitch)*cos(shoulder_roll);
J(5,3)=1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch);
}


void getJacobianBaseWrist(std::vector<double> q,Eigen::Matrix<double,6,7> &J)
 { 
 J.setZero(); 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];


J(0,0)=-0.30745*cos(shoulder_pitch)*cos(shoulder_roll);
J(0,1)=0.30745*sin(shoulder_pitch)*sin(shoulder_roll);
J(0,2)=0;
J(0,3)=0;
J(0,4)=0;
J(1,0)=0;
J(1,1)=0.30745*sin(shoulder_pitch)**2*cos(shoulder_roll) + 0.30745*cos(shoulder_pitch)**2*cos(shoulder_roll);
J(1,2)=0;
J(1,3)=0;
J(1,4)=0;
J(2,0)=0.30745*sin(shoulder_pitch)*cos(shoulder_roll);
J(2,1)=0.30745*sin(shoulder_roll)*cos(shoulder_pitch);
J(2,2)=0;
J(2,3)=0;
J(2,4)=0;
J(3,0)=0;
J(3,1)=1.0*cos(shoulder_pitch);
J(3,2)=1.0*sin(shoulder_pitch)*cos(shoulder_roll);
J(3,3)=-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw);
J(3,4)=1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll);
J(4,0)=1.00000000000000;
J(4,1)=0;
J(4,2)=-1.0*sin(shoulder_roll);
J(4,3)=1.0*cos(arm_yaw)*cos(shoulder_roll);
J(4,4)=1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch);
J(5,0)=0;
J(5,1)=-1.0*sin(shoulder_pitch);
J(5,2)=1.0*cos(shoulder_pitch)*cos(shoulder_roll);
J(5,3)=1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch);
J(5,4)=1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll);
}


void getJacobianBaseWrist_hand(std::vector<double> q,Eigen::Matrix<double,6,7> &J)
 { 
 J.setZero(); 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];


J(0,0)=-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll);
J(0,1)=1.0*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll))*sin(shoulder_pitch);
J(0,2)=-1.0*(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll))*sin(shoulder_roll) - 1.0*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll))*cos(shoulder_pitch)*cos(shoulder_roll);
J(0,3)=1.0*(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(arm_yaw)*cos(shoulder_roll) + (-1.0*sin(arm_yaw)*sin(shoulder_pitch) - 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch));
J(0,4)=(-1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch)) + (-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*(1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch));
J(0,5)=0;
J(1,0)=0;
J(1,1)=-1.0*(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) - 0.30745*sin(shoulder_pitch)*cos(shoulder_roll))*sin(shoulder_pitch) - 1.0*(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll))*cos(shoulder_pitch);
J(1,2)=1.0*(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) - 0.30745*sin(shoulder_pitch)*cos(shoulder_roll))*cos(shoulder_pitch)*cos(shoulder_roll) - 1.0*(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll))*sin(shoulder_pitch)*cos(shoulder_roll);
J(1,3)=(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch)) + (-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*(1.0*sin(arm_yaw)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw));
J(1,4)=(-1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll)) + (-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll));
J(1,5)=0;
J(2,0)=0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) + 0.30745*sin(shoulder_pitch)*cos(shoulder_roll);
J(2,1)=1.0*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll))*cos(shoulder_pitch);
J(2,2)=1.0*(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) - 0.30745*sin(shoulder_pitch)*cos(shoulder_roll))*sin(shoulder_roll) + 1.0*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll))*sin(shoulder_pitch)*cos(shoulder_roll);
J(2,3)=-1.0*(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*cos(arm_yaw)*cos(shoulder_roll) + (-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch));
J(2,4)=(-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*(-1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(shoulder_roll)*cos(elbow_pitch)) + (1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*(-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch));
J(2,5)=0;
J(3,0)=0;
J(3,1)=1.0*cos(shoulder_pitch);
J(3,2)=1.0*sin(shoulder_pitch)*cos(shoulder_roll);
J(3,3)=-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw);
J(3,4)=1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll);
J(3,5)=-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*cos(forearm_yaw);
J(4,0)=1.00000000000000;
J(4,1)=0;
J(4,2)=-1.0*sin(shoulder_roll);
J(4,3)=1.0*cos(arm_yaw)*cos(shoulder_roll);
J(4,4)=1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch);
J(4,5)=-1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*sin(forearm_yaw) + 1.0*cos(arm_yaw)*cos(forearm_yaw)*cos(shoulder_roll);
J(5,0)=0;
J(5,1)=-1.0*sin(shoulder_pitch);
J(5,2)=1.0*cos(shoulder_pitch)*cos(shoulder_roll);
J(5,3)=1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch);
J(5,4)=1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll);
J(5,5)=-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*cos(forearm_yaw);
}


