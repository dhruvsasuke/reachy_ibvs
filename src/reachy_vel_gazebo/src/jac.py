#!/usr/bin/env python
#To calculate jacobian
import numpy as numpy
import jacobian_function


    ##For testing
def main():
    print(jacobian_function.calc_jack(0,0,0,0,0,0))


if __name__=='__main__':
    main()



##############
'''
roslaunch reachy_vel_gazebo reachy_gazebo.launch 
roslaunch reachy_vel_control arm_controller.launch 
rostopic pub /reachy/shoulder_pitch_velocity_controller/command std_msgs/Float64 "data: 0.00" 
roslaunch reachy_vel_moveit planning_context.launch
rosrun reachy_vel_moveit jacobian 
rosrun image_view image_view image:=/camera/rgb/image_raw
'''