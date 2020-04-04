# 

So far this repo can generate symbolic transformation matrices and Jacobians for a serial chain from a URDF

```
	rosrun symbolic_models sym_kinematics.py -r "right_arm_base_link" -t "right_l6"
```

Description: This program obtains the analytic value for the jacobian
and transformation matrix from the root joint to the tip joint for a robot
description on the parameter server.This program supports prismatic and
revloute joints only.
The output are saved as functions in C++ (.cpp  .h matrix library Eigen) and matlab .m. \n \n

To Do: Add support for different joints, print out matrices in python format, add dynamics

## Argument is not an mpz 
It seems that there is an issue with the Python described [https://github.com/sympy/sympy/issues/12895](here) 
The quick fix is to set an environment variable in current session using:
```  export MPMATH_NOGMPY=1```