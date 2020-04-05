#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <eigen_conversions/eigen_msg.h>
#include "Transforms_base_hand_ball.h"
#include "Transforms_base_hand_ball.cpp"

std::vector<double> joint_states(7);
Eigen::Transform<double, 3, Eigen::Affine> trans;

void Callback(const sensor_msgs::JointState &msg){
    joint_states = msg.position;
    getTransformBaseHand_ball(joint_states, trans);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "transform");
    ros::NodeHandle n;
    ros::Rate rate(10);
    ros::Subscriber sub = n.subscribe("reachy/joint_states", 100, Callback);
    ros::Publisher pub = n.advertise<geometry_msgs::Transform>("transform_topic", 100);
    geometry_msgs::Transform trans_msg;

    while(ros::ok()){
        tf::transformEigenToMsg(trans, trans_msg);
        pub.publish(trans_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
