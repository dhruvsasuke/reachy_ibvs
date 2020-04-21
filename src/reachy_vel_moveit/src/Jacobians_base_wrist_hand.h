void getJacobianBaseShoulder(std::vector<double> q,Eigen::Matrix<double,6,6> &J);
void getJacobianBaseShoulder_to_arm(std::vector<double> q,Eigen::Matrix<double,6,6> &J);
void getJacobianBaseUpper_arm(std::vector<double> q,Eigen::Matrix<double,6,6> &J);
void getJacobianBaseForearm(std::vector<double> q,Eigen::Matrix<double,6,6> &J);
void getJacobianBaseWrist(std::vector<double> q,Eigen::Matrix<double,6,6> &J);
void getJacobianBaseWrist_hand(std::vector<double> q,Eigen::Matrix<double,6,6> &J);
