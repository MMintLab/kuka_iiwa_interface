#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from victor_hardware_interface_msgs.msg import MotionStatus
from victor_hardware_interface_msgs.msg import Robotiq3FingerStatus
from threading import Lock
from math import radians


def compute_finger_angles(control):
    # the control is from 0 to 1, however, the relationship takes 0 to 255
    g = control * 255
    max_angle = [70.0, 90.0, 43.0]
    min_3 = -55.0
    m1 = max_angle[0] / 140.0
    m2 = max_angle[1] / 100.0

    # http://motion.pratt.duke.edu/papers/IUCS-TR711-Franchi-gripper.pdf
    # Based on the relationship from the documentation, set each joint angle based on the "phase" of the motion
    if g <= 110.0:
        theta1 = m1 * g
        theta2 = 0
        theta3 = -m1 * g
    elif 110.0 < g <= 140.0:
        theta1 = m1 * g
        theta2 = 0
        theta3 = min_3
    elif 140.0 < g <= 240.0:
        theta1 = max_angle[0]
        theta2 = m2 * (g - 140)
        theta3 = min_3
    else:
        theta1 = max_angle[0]
        theta2 = max_angle[1]
        theta3 = min_3

    return [radians(theta1), radians(theta2), radians(theta3)]


def compute_scissor_angle(control):
    # 0 corresponds to fully open at -16 degrees, 1 is fully closed with at +10 degrees
    return radians(26.0 * control - 16.0)


class ThanosJointStatePublisher:
    def __init__(self):
        # We don't want to do this programatically, because the callback functions are using assuming a specific order
        # for the joints. We could change the callback functions to remove this assumption, but that would increase the
        # complexity of the callback functions unnecessarily
        self.joint_names = [
            'thanos_kuka_joint_1',
            'thanos_kuka_joint_2',
            'thanos_kuka_joint_3',
            'thanos_kuka_joint_4',
            'thanos_kuka_joint_5',
            'thanos_kuka_joint_6',
            'thanos_kuka_joint_7',
            ]


        # Setup the output message with default values
        self.joint_state_lock = Lock()
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = [0] * len(self.joint_names)
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = [0] * len(self.joint_names)

        # Setup the publishers and subscribers that will be used
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.thanos_arm_sub = rospy.Subscriber("left_arm/motion_status", MotionStatus,
                                               self.left_arm_motion_status_callback)

    def run(self, loop_rate):
        rate = rospy.Rate(loop_rate)
        while not rospy.is_shutdown():
            self.publish_joint_values()
            rate.sleep()

    def left_arm_motion_status_callback(self, motion_status):
        self.set_arm_position_values(motion_status, offset=0)
        self.set_arm_effort_values(motion_status, offset=0)

    def set_arm_position_values(self, motion_status, offset):
        with self.joint_state_lock:
            self.joint_state_msg.position[offset + 0] = motion_status.measured_joint_position.joint_1
            self.joint_state_msg.position[offset + 1] = motion_status.measured_joint_position.joint_2
            self.joint_state_msg.position[offset + 2] = motion_status.measured_joint_position.joint_3
            self.joint_state_msg.position[offset + 3] = motion_status.measured_joint_position.joint_4
            self.joint_state_msg.position[offset + 4] = motion_status.measured_joint_position.joint_5
            self.joint_state_msg.position[offset + 5] = motion_status.measured_joint_position.joint_6
            self.joint_state_msg.position[offset + 6] = motion_status.measured_joint_position.joint_7

    def set_arm_effort_values(self, motion_status, offset):
        # We use measured joint torque for now. As Kuka mentioned, it is the currently measured "raw" torque sensor
        # data. There is an estimated_external_torque message, which is the current external torque sensor data for
        # this robot.
        with self.joint_state_lock:
            self.joint_state_msg.effort[offset + 0] = motion_status.measured_joint_torque.joint_1
            self.joint_state_msg.effort[offset + 1] = motion_status.measured_joint_torque.joint_2
            self.joint_state_msg.effort[offset + 2] = motion_status.measured_joint_torque.joint_3
            self.joint_state_msg.effort[offset + 3] = motion_status.measured_joint_torque.joint_4
            self.joint_state_msg.effort[offset + 4] = motion_status.measured_joint_torque.joint_5
            self.joint_state_msg.effort[offset + 5] = motion_status.measured_joint_torque.joint_6
            self.joint_state_msg.effort[offset + 6] = motion_status.measured_joint_torque.joint_7

    def publish_joint_values(self):
        with self.joint_state_lock:
            self.joint_state_msg.header.stamp = rospy.Time.now()
            self.joint_state_pub.publish(self.joint_state_msg)


if __name__ == '__main__':
    rospy.init_node('thanos_joint_state_publisher')
    rospy.loginfo('Starting the thanos joint state broadcaster...')

    rate = rospy.get_param("~rate", 10.0)
    pub = ThanosJointStatePublisher()
    pub.run(rate)
