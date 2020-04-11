#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <eigen_conversions/eigen_msg.h>
#include "Jacobians_base_hand_ball.h"
#include "Jacobians_base_hand_ball.cpp"
#include "Jacobians_base_wrist_hand.h"
#include "Jacobians_base_wrist_hand.h"
std::vector<double> joint_values(6);
Eigen::Matrix<double,6,7> jacobian;

void Callback(const sensor_msgs::JointState &msg){
  //std::cout<<"callback \n";
  for(int i=0; i<joint_values.size(); i++)
    joint_values[i] = msg.position[i];
}

int main(int argc, char** argv){
  ros::init(argc, argv, "jacobian");
  ros::NodeHandle n, nh;
  ros::Rate rate(20);

  ros::Subscriber sub = n.subscribe("/reachy/joint_states", 100, Callback);
  ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("jacobian_topic", 100);
  std_msgs::Float64MultiArray data;
  data.layout.data_offset = 0;

  while(ros::ok()){

    data.data.clear();
    getJacobianBaseWrist_hand(joint_values,jacobian);    
    std_msgs::Float64MultiArray data;
    tf::matrixEigenToMsg(jacobian, data);
    pub.publish(data);
    ros::spinOnce();
    rate.sleep();
  }  
  return 0;  

}

// int main(int argc, char** argv){
//   ros::init(argc, argv, "jacobian");

//   ros::NodeHandle n, nh;
//   ros::Rate rate(10);
//   ros::Subscriber sub = n.subscribe("/reachy/joint_states", 100, Callback);
//   ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("jacobian_topic", 100);
//   std_msgs::Float64MultiArray data;
//   data.layout.data_offset = 0;

//   // Import Robot Model from Robot Description on Parameter Server

//   robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//   robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//   ROS_INFO("Model Frame: %s", kinematic_model->getModelFrame().c_str());

//   // Get the Robot_State from Robot Model that maintains current configuration of Robot
//   //  and store the Joints names and angles in the Joint Group 'arm'

//   robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
//   kinematic_state->setToDefaultValues();
//   const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
//   const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

//   kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
//   for(int i=0; i<joint_names.size(); i++)
//       ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]); 

//   // Setting joint positions by setJointGroupPositions()
  
//   while(ros::ok()){

//     //std::cout<<"ros \n";

//     data.data.clear();
//     kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

//     // Get the Jacobian
//     // ^^^^^^^^^^^^^^^^
//     // We can also get the Jacobian from the :moveit_core:`RobotState`.
//     Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
//     Eigen::MatrixXd jacobian;
//     kinematic_state->getJacobian(joint_model_group,
//                                   kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
//                                   reference_point_position, jacobian);

//     std_msgs::Float64MultiArray data;
//     tf::matrixEigenToMsg(jacobian, data);

//     pub.publish(data);

//     ros::spinOnce();
//     rate.sleep();
//   }  
//   return 0;
// }