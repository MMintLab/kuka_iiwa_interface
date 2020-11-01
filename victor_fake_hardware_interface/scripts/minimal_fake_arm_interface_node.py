#!/usr/bin/env python

import rospy
from victor_fake_hardware_interface.minimal_fake_arm_interface import MinimalFakeArmInterface, \
    MinimalFakeGripperInterface


def main():
    rospy.init_node("minimal_fake_arm_interface")

    interfaces = {}

    arm_names = ["left_arm", "right_arm"]

    for arm in arm_names:
        interfaces[arm] = MinimalFakeArmInterface(arm_name=arm,
                                                  control_mode_status_topic=arm + "/control_mode_status",
                                                  get_control_mode_service_topic=arm + "/get_control_mode_service",
                                                  set_control_mode_service_topic=arm + "/set_control_mode_service",
                                                  motion_command_topic=arm + "/motion_command",
                                                  motion_status_topic=arm + "/motion_status")
        interfaces[arm + "/gripper"] = MinimalFakeGripperInterface(gripper_command_topic=arm + "/gripper_command",
                                                                   gripper_status_topic=arm + "/gripper_status")

    for arm in arm_names:
        interfaces[arm].start_feedback_threads()
        interfaces[arm + "/gripper"].start_feedback_threads()

    rospy.loginfo("Publishing data...")
    rospy.spin()

    for arm in arm_names:
        interfaces[arm].join_feedback_threads()
        interfaces[arm + "/gripper"].join_feedback_threads()


if __name__ == "__main__":
    main()
