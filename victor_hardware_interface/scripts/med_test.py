#! /usr/bin/env python

import rospy
import numpy as np
from victor_hardware_interface_msgs.msg import ControlMode, MotionStatus, MotionCommand
from victor_hardware_interface.victor_utils import get_control_mode_params, list_to_jvq, jvq_to_list
from sensor_msgs.msg import JointState
import pdb

rospy.init_node("med_motion_test")
arm_command_pub = rospy.Publisher("/victor/left_arm/motion_command", MotionCommand, queue_size=10)

current_pose = None
def get_current_pose_cb(js):
    global current_pose
    current_pose = js.position
rospy.Subscriber("/victor/joint_states", JointState, get_current_pose_cb)

positions = [
    [0,0,0,0,0,0,0],
    [0,1.0,0,1.0,0,1.0,0],
    [-0.5,1.0,0,1.0,0,1.0,0],
    [0.5,1.2,-0.5,-1.0,-0.3,0,0],
    [0,0,0,0,0,0,0],
]
velocities = [0,0,0,0,0,0,0]

# Wait to get current pose:
while current_pose is None:
    rospy.sleep(0.1)

r = rospy.Rate(5)
i = 0
while not rospy.is_shutdown():
    cmd = MotionCommand(joint_position=list_to_jvq(positions[i]),
                        joint_velocity=list_to_jvq(velocities),
                        control_mode=ControlMode(ControlMode.JOINT_POSITION))
    cmd.header.stamp = rospy.Time.now()
    arm_command_pub.publish(cmd)

    if np.linalg.norm(np.array(positions[i]) - np.array(current_pose)) < 0.001:
        i = i+1
        if i == len(positions):
            break
    
    r.sleep()
