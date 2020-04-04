void getTransformBaseShoulder(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T);
void getTransformBaseShoulder_to_arm(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T);
void getTransformBaseUpper_arm(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T);
void getTransformBaseForearm(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T);
void getTransformBaseWrist(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T);
void getTransformBaseWrist_hand(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T);
void getTransformBaseHand_ball(std::vector<double> q,Eigen::Transform<double,3,Eigen::Affine> & T);
