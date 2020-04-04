void getTransformBaseShoulder(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T)
 { 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];
double wrist_roll=q[6];


T(0,0)=1.0*cos(shoulder_pitch);
T(0,1)=0;
T(0,2)=1.0*sin(shoulder_pitch);
T(0,3)=0;
T(1,0)=0;
T(1,1)=1.00000000000000;
T(1,2)=0;
T(1,3)=-0.0400000000000000;
T(2,0)=-1.0*sin(shoulder_pitch);
T(2,1)=0;
T(2,2)=1.0*cos(shoulder_pitch);
T(2,3)=0;
T(3,0)=0;
T(3,1)=0;
T(3,2)=0;
T(3,3)=1.00000000000000;
}


void getTransformBaseShoulder_to_arm(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T)
 { 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];
double wrist_roll=q[6];


T(0,0)=1.0*cos(shoulder_pitch);
T(0,1)=1.0*sin(shoulder_pitch)*sin(shoulder_roll);
T(0,2)=1.0*sin(shoulder_pitch)*cos(shoulder_roll);
T(0,3)=0;
T(1,0)=0;
T(1,1)=1.0*cos(shoulder_roll);
T(1,2)=-1.0*sin(shoulder_roll);
T(1,3)=-0.0400000000000000;
T(2,0)=-1.0*sin(shoulder_pitch);
T(2,1)=1.0*sin(shoulder_roll)*cos(shoulder_pitch);
T(2,2)=1.0*cos(shoulder_pitch)*cos(shoulder_roll);
T(2,3)=0;
T(3,0)=0;
T(3,1)=0;
T(3,2)=0;
T(3,3)=1.00000000000000;
}


void getTransformBaseUpper_arm(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T)
 { 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];
double wrist_roll=q[6];


T(0,0)=1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch);
T(0,1)=-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw);
T(0,2)=1.0*sin(shoulder_pitch)*cos(shoulder_roll);
T(0,3)=0;
T(1,0)=1.0*sin(arm_yaw)*cos(shoulder_roll);
T(1,1)=1.0*cos(arm_yaw)*cos(shoulder_roll);
T(1,2)=-1.0*sin(shoulder_roll);
T(1,3)=-0.0400000000000000;
T(2,0)=1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw);
T(2,1)=1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch);
T(2,2)=1.0*cos(shoulder_pitch)*cos(shoulder_roll);
T(2,3)=0;
T(3,0)=0;
T(3,1)=0;
T(3,2)=0;
T(3,3)=1.00000000000000;
}


void getTransformBaseForearm(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T)
 { 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];
double wrist_roll=q[6];


T(0,0)=1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll);
T(0,1)=-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw);
T(0,2)=1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll);
T(0,3)=-0.30745*sin(shoulder_pitch)*cos(shoulder_roll);
T(1,0)=1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll);
T(1,1)=1.0*cos(arm_yaw)*cos(shoulder_roll);
T(1,2)=1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch);
T(1,3)=0.30745*sin(shoulder_roll) - 0.04;
T(2,0)=1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll);
T(2,1)=1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch);
T(2,2)=1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll);
T(2,3)=-0.30745*cos(shoulder_pitch)*cos(shoulder_roll);
T(3,0)=0;
T(3,1)=0;
T(3,2)=0;
T(3,3)=1.00000000000000;
}


void getTransformBaseWrist(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T)
 { 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];
double wrist_roll=q[6];


T(0,0)=1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*sin(forearm_yaw);
T(0,1)=-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*cos(forearm_yaw);
T(0,2)=1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll);
T(0,3)=-0.30745*sin(shoulder_pitch)*cos(shoulder_roll);
T(1,0)=1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*cos(forearm_yaw) + 1.0*sin(forearm_yaw)*cos(arm_yaw)*cos(shoulder_roll);
T(1,1)=-1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*sin(forearm_yaw) + 1.0*cos(arm_yaw)*cos(forearm_yaw)*cos(shoulder_roll);
T(1,2)=1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch);
T(1,3)=0.30745*sin(shoulder_roll) - 0.04;
T(2,0)=1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*sin(forearm_yaw);
T(2,1)=-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*cos(forearm_yaw);
T(2,2)=1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll);
T(2,3)=-0.30745*cos(shoulder_pitch)*cos(shoulder_roll);
T(3,0)=0;
T(3,1)=0;
T(3,2)=0;
T(3,3)=1.00000000000000;
}


void getTransformBaseWrist_hand(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T)
 { 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];
double wrist_roll=q[6];


T(0,0)=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*sin(forearm_yaw))*cos(wrist_pitch) - 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*sin(wrist_pitch);
T(0,1)=-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*cos(forearm_yaw);
T(0,2)=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*sin(forearm_yaw))*sin(wrist_pitch) + 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*cos(wrist_pitch);
T(0,3)=-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) - 0.30745*sin(shoulder_pitch)*cos(shoulder_roll);
T(1,0)=1.0*(1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*cos(forearm_yaw) + 1.0*sin(forearm_yaw)*cos(arm_yaw)*cos(shoulder_roll))*cos(wrist_pitch) - 1.0*(1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch))*sin(wrist_pitch);
T(1,1)=-1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*sin(forearm_yaw) + 1.0*cos(arm_yaw)*cos(forearm_yaw)*cos(shoulder_roll);
T(1,2)=1.0*(1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*cos(forearm_yaw) + 1.0*sin(forearm_yaw)*cos(arm_yaw)*cos(shoulder_roll))*sin(wrist_pitch) + 1.0*(1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch))*cos(wrist_pitch);
T(1,3)=-0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll) - 0.04;
T(2,0)=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*sin(forearm_yaw))*cos(wrist_pitch) - 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*sin(wrist_pitch);
T(2,1)=-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*cos(forearm_yaw);
T(2,2)=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*sin(forearm_yaw))*sin(wrist_pitch) + 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(wrist_pitch);
T(2,3)=-0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll);
T(3,0)=0;
T(3,1)=0;
T(3,2)=0;
T(3,3)=1.00000000000000;
}


void getTransformBaseHand_ball(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T)
 { 
double shoulder_pitch=q[0];
double shoulder_roll=q[1];
double arm_yaw=q[2];
double elbow_pitch=q[3];
double forearm_yaw=q[4];
double wrist_pitch=q[5];
double wrist_roll=q[6];


T(0,0)=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*sin(forearm_yaw))*cos(wrist_pitch) - 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*sin(wrist_pitch);
T(0,1)=1.0*(1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*sin(forearm_yaw))*sin(wrist_pitch) + 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*cos(wrist_pitch))*sin(wrist_roll) + 1.0*(-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*cos(forearm_yaw))*cos(wrist_roll);
T(0,2)=1.0*(1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*sin(forearm_yaw))*sin(wrist_pitch) + 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*cos(wrist_pitch))*cos(wrist_roll) - 1.0*(-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*cos(forearm_yaw))*sin(wrist_roll);
T(0,3)=-0.03243*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*sin(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(-1.0*sin(arm_yaw)*cos(shoulder_pitch) + 1.0*sin(shoulder_pitch)*sin(shoulder_roll)*cos(arm_yaw))*sin(forearm_yaw))*sin(wrist_pitch) - 0.03243*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) + 1.0*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll))*cos(wrist_pitch) - 0.22425*(1.0*sin(arm_yaw)*sin(shoulder_pitch)*sin(shoulder_roll) + 1.0*cos(arm_yaw)*cos(shoulder_pitch))*sin(elbow_pitch) - 0.22425*sin(shoulder_pitch)*cos(elbow_pitch)*cos(shoulder_roll) - 0.30745*sin(shoulder_pitch)*cos(shoulder_roll);
T(1,0)=1.0*(1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*cos(forearm_yaw) + 1.0*sin(forearm_yaw)*cos(arm_yaw)*cos(shoulder_roll))*cos(wrist_pitch) - 1.0*(1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch))*sin(wrist_pitch);
T(1,1)=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*cos(forearm_yaw) + 1.0*sin(forearm_yaw)*cos(arm_yaw)*cos(shoulder_roll))*sin(wrist_pitch) + 1.0*(1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch))*cos(wrist_pitch))*sin(wrist_roll) + 1.0*(-1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*sin(forearm_yaw) + 1.0*cos(arm_yaw)*cos(forearm_yaw)*cos(shoulder_roll))*cos(wrist_roll);
T(1,2)=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*cos(forearm_yaw) + 1.0*sin(forearm_yaw)*cos(arm_yaw)*cos(shoulder_roll))*sin(wrist_pitch) + 1.0*(1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch))*cos(wrist_pitch))*cos(wrist_roll) - 1.0*(-1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*sin(forearm_yaw) + 1.0*cos(arm_yaw)*cos(forearm_yaw)*cos(shoulder_roll))*sin(wrist_roll);
T(1,3)=-0.03243*(1.0*(1.0*sin(arm_yaw)*cos(elbow_pitch)*cos(shoulder_roll) + 1.0*sin(elbow_pitch)*sin(shoulder_roll))*cos(forearm_yaw) + 1.0*sin(forearm_yaw)*cos(arm_yaw)*cos(shoulder_roll))*sin(wrist_pitch) - 0.03243*(1.0*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) - 1.0*sin(shoulder_roll)*cos(elbow_pitch))*cos(wrist_pitch) - 0.22425*sin(arm_yaw)*sin(elbow_pitch)*cos(shoulder_roll) + 0.22425*sin(shoulder_roll)*cos(elbow_pitch) + 0.30745*sin(shoulder_roll) - 0.04;
T(2,0)=1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*sin(forearm_yaw))*cos(wrist_pitch) - 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*sin(wrist_pitch);
T(2,1)=1.0*(1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*sin(forearm_yaw))*sin(wrist_pitch) + 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(wrist_pitch))*sin(wrist_roll) + 1.0*(-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*cos(forearm_yaw))*cos(wrist_roll);
T(2,2)=1.0*(1.0*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*sin(forearm_yaw))*sin(wrist_pitch) + 1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(wrist_pitch))*cos(wrist_roll) - 1.0*(-1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*sin(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*cos(forearm_yaw))*sin(wrist_roll);
T(2,3)=-0.03243*(1.0*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*cos(elbow_pitch) - 1.0*sin(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(forearm_yaw) + 1.0*(1.0*sin(arm_yaw)*sin(shoulder_pitch) + 1.0*sin(shoulder_roll)*cos(arm_yaw)*cos(shoulder_pitch))*sin(forearm_yaw))*sin(wrist_pitch) - 0.03243*(1.0*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) + 1.0*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll))*cos(wrist_pitch) - 0.22425*(1.0*sin(arm_yaw)*sin(shoulder_roll)*cos(shoulder_pitch) - 1.0*sin(shoulder_pitch)*cos(arm_yaw))*sin(elbow_pitch) - 0.22425*cos(elbow_pitch)*cos(shoulder_pitch)*cos(shoulder_roll) - 0.30745*cos(shoulder_pitch)*cos(shoulder_roll);
T(3,0)=0;
T(3,1)=0;
T(3,2)=0;
T(3,3)=1.00000000000000;
}


