import rospy
import time
from std_msgs.msg import Float64


pub1 = rospy.Publisher("/reachy/shoulder_pitch_velocity_controller/command", Float64, queue_size=10)
pub2 = rospy.Publisher("/reachy/shoulder_roll_velocity_controller/command", Float64, queue_size=10)
pub3 = rospy.Publisher("/reachy/arm_yaw_velocity_controller/command", Float64, queue_size=10)
pub4 = rospy.Publisher("/reachy/elbow_pitch_velocity_controller/command", Float64, queue_size=10)
pub5 = rospy.Publisher("/reachy/forearm_yaw_velocity_controller/command", Float64, queue_size=10)
pub6 = rospy.Publisher("/reachy/wrist_pitch_velocity_controller/command", Float64, queue_size=10)    

def init():
    global pub1, pub2, pub3, pub4, pub5, pub6
    start = time.time()
    while ((time.time() - start)<12):
        pub1.publish(0.05)
        pub4.publish(-0.1)
        pub6.publish(0.04)
    pub1.publish(0.0)
    pub4.publish(0.0)
    pub6.publish(0.0)


if __name__ == "__main__":
    rospy.init_node("initialise", anonymous="True")    
    init()
