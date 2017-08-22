#!/usr/bin/env python

# [Create a window]

# Python imports
import sys
import signal
import time
import math
import copy
from functools import partial

# Qt imports
from PyQt5.QtCore import Qt
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

# ROS imports
import rospkg
import rospy
from victor_hardware_interface.msg import Robotiq3FingerCommand, MotionCommand, MotionStatus, Robotiq3FingerStatus, \
    ControlModeParameters, ControlMode
from victor_hardware_interface.srv import SetControlMode, GetControlMode

finger_range_discretization = 1000
arm_joint_limit_margin = 1
block_command = False

joint_limits = {'joint_1': (-170, 170),
                'joint_2': (-120, 120),
                'joint_3': (-170, 170),
                'joint_4': (-120, 120),
                'joint_5': (-170, 170),
                'joint_6': (-120, 120),
                'joint_7': (-170, 170)}

joint_limits_with_margin = {joint_name: (lower + arm_joint_limit_margin,
                                      upper - arm_joint_limit_margin)
                         for (joint_name, (lower, upper)) in joint_limits.iteritems()}

joint_names = ['joint_' + str(i) for i in range(1,8)]

def clip(value, joint_name):
    min_lim, max_lim = joint_limits_with_margin[joint_name]
    return min(max(value, min_lim), max_lim)


class Widget(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
  
        self.v_layout = QGridLayout()

        self.left_arm = Arm(self.v_layout, 'left_arm', 0)
        self.right_arm = Arm(self.v_layout, 'right_arm', 1)

        self.setLayout(self.v_layout)


class Arm:
    def __init__(self, parent_layout, arm_name, box_row):
        self.name = arm_name

        self.v_layout = QGridLayout()
        self.col_num = 11
        self.row_count = [0] * self.col_num

        self.reset_arm_to_zero_button = self.init_button('Home the Arm', 0, self.reset_arm_command_to_zero)
        self.reset_arm_slider_to_current_value_button = self.init_button('Reset the Sliders', 1,
                                                                         self.reset_arm_slider_to_current_value)
        self.open_gripper_button = self.init_button('Open the Gripper', 2, self.open_gripper_command)
        self.close_gripper_button = self.init_button('Close the Gripper', 4, self.close_gripper_command)
        self.reset_speed_force_button = self.init_button('Reset Speed and Force', 6, self.reset_speed_force_command)
        self.reset_gripper_position_slider_to_current_value_button = self.init_button('Reset the Sliders', 8,
                                                                                      self.reset_gripper_position_slider_to_current_value)

        self.synchonize_label = self.init_label('Synchronize', 8)
        self.finger_same_position_checkbox = self.init_checkbox('', 8, self.finger_same_position_checkbox_changed, 0)
        self.finger_same_speed_checkbox = self.init_checkbox('', 8, self.finger_same_speed_checkbox_changed, 1)
        self.finger_same_force_checkbox = self.init_checkbox('', 8, self.finger_same_force_checkbox_changed, 1)

        self.joint_labels = {}
        self.joint_sliders = {}
        self.joint_textboxes = {}




        for joint_name in joint_names:
            slider_cb = partial(self.joint_slider_moved, joint_name = joint_name)
            text_cb = partial(self.joint_textbox_modified, joint_name = joint_name)

            label = 'Joint ' + joint_name[-1] + ' Command'
            
            self.joint_labels[joint_name] = self.init_label(label, 0)
            self.joint_sliders[joint_name] = self.init_slider(joint_limits_with_margin[joint_name],
                                                              0, slider_cb)
            self.joint_textboxes[joint_name] = self.init_textbox('0', 1, text_cb, 1)



        

        self.finger_a_pos_label = self.init_label('Finger A Command Position', 2)
        self.finger_a_pos_slider = self.init_slider((0, finger_range_discretization), 2, self.finger_a_pos_slider_moved)
        self.finger_a_spe_label = self.init_label('Finger A Command Speed', 2)
        self.finger_a_spe_slider = self.init_slider((0, finger_range_discretization), 2, self.finger_a_spe_slider_moved, finger_range_discretization)
        self.finger_a_frc_label = self.init_label('Finger A Command Force', 2)
        self.finger_a_frc_slider = self.init_slider((0, finger_range_discretization), 2, self.finger_a_frc_slider_moved, finger_range_discretization)
        self.finger_a_pos_textbox = self.init_textbox('0.000', 3, self.finger_a_pos_textbox_modified, 2)
        self.finger_a_spe_textbox = self.init_textbox('1.000', 3, self.finger_a_spe_textbox_modified, 1)
        self.finger_a_frc_textbox = self.init_textbox('1.000', 3, self.finger_a_frc_textbox_modified, 1)

        self.finger_b_pos_label = self.init_label('Finger B Command Position', 4)
        self.finger_b_pos_slider = self.init_slider((0, finger_range_discretization), 4, self.finger_b_pos_slider_moved)
        self.finger_b_spe_label = self.init_label('Finger B Command Speed', 4)
        self.finger_b_spe_slider = self.init_slider((0, finger_range_discretization), 4, self.finger_b_spe_slider_moved, finger_range_discretization)
        self.finger_b_frc_label = self.init_label('Finger B Command Force', 4)
        self.finger_b_frc_slider = self.init_slider((0, finger_range_discretization), 4, self.finger_b_frc_slider_moved, finger_range_discretization)
        self.finger_b_pos_textbox = self.init_textbox('0.000', 5, self.finger_b_pos_textbox_modified, 2)
        self.finger_b_spe_textbox = self.init_textbox('1.000', 5, self.finger_b_spe_textbox_modified, 1)
        self.finger_b_frc_textbox = self.init_textbox('1.000', 5, self.finger_b_frc_textbox_modified, 1)

        self.finger_c_pos_label = self.init_label('Finger C Command Position', 6)
        self.finger_c_pos_slider = self.init_slider((0, finger_range_discretization), 6, self.finger_c_pos_slider_moved)
        self.finger_c_spe_label = self.init_label('Finger C Command Speed', 6)
        self.finger_c_spe_slider = self.init_slider((0, finger_range_discretization), 6, self.finger_c_spe_slider_moved, finger_range_discretization)
        self.finger_c_frc_label = self.init_label('Finger C Command Force', 6)
        self.finger_c_frc_slider = self.init_slider((0, finger_range_discretization), 6, self.finger_c_frc_slider_moved, finger_range_discretization)
        self.finger_c_pos_textbox = self.init_textbox('0.000', 7, self.finger_c_pos_textbox_modified, 2)
        self.finger_c_spe_textbox = self.init_textbox('1.000', 7, self.finger_c_spe_textbox_modified, 1)
        self.finger_c_frc_textbox = self.init_textbox('1.000', 7, self.finger_c_frc_textbox_modified, 1)

        self.scissor_pos_label = self.init_label('Scissor Command Position', 9, 1)
        self.scissor_pos_slider = self.init_slider((0, finger_range_discretization), 9, self.scissor_pos_slider_moved)
        self.scissor_spe_label = self.init_label('Scissor Command Speed', 9)
        self.scissor_spe_slider = self.init_slider((0, finger_range_discretization), 9, self.scissor_spe_slider_moved,
                                                   finger_range_discretization)
        self.scissor_frc_label = self.init_label('Scissor Command Force', 9)
        self.scissor_frc_slider = self.init_slider((0, finger_range_discretization), 9, self.scissor_frc_slider_moved,
                                                   finger_range_discretization)
        self.scissor_pos_textbox = self.init_textbox('0.000', 10, self.scissor_pos_textbox_modified, 2)
        self.scissor_spe_textbox = self.init_textbox('1.000', 10, self.scissor_spe_textbox_modified, 1)
        self.scissor_frc_textbox = self.init_textbox('1.000', 10, self.scissor_frc_textbox_modified, 1)

        self.control_mode_label = self.init_label('Control Mode', 2, 1)
        self.control_mode_combobox = self.init_combobox(['JOINT_POSITION', 'CARTESIAN_POSE', 'JOINT_IMPEDANCE', 'CARTESIAN_IMPEDANCE'], 2, self.change_control_mode, 0)

        self.finger_command = Robotiq3FingerCommand()
        self.arm_command = MotionCommand()
        self.arm_status = MotionStatus()
        self.gripper_status = Robotiq3FingerStatus()

        self.arm_status_updated = False
        self.gripper_status_updated = False

        self.finger_command.finger_a_command.speed = 1.0
        self.finger_command.finger_b_command.speed = 1.0
        self.finger_command.finger_c_command.speed = 1.0
        self.finger_command.scissor_command.speed = 1.0

        self.finger_command_publisher = rospy.Publisher(self.name + '/gripper_command', Robotiq3FingerCommand, queue_size=10)
        self.arm_command_publisher = rospy.Publisher(self.name + '/motion_command', MotionCommand, queue_size=10)

        self.gripper_status_subscriber = rospy.Subscriber(self.name + '/gripper_status', Robotiq3FingerStatus, self.gripper_status_callback)
        self.arm_status_subscriber = rospy.Subscriber(self.name + '/motion_status', MotionStatus, self.arm_status_callback)

        self.fingers_same_position = False
        self.fingers_same_speed = False
        self.fingers_same_force = False

        self.groupbox = QGroupBox(self.name)
        self.groupbox.setLayout(self.v_layout)
        self.groupbox.setFlat(False)
        parent_layout.addWidget(self.groupbox, box_row, 0)

        print 'Getting', self.name, 'current joint values ...',
        sys.stdout.flush()
        while (not self.arm_status_updated) or (not self.gripper_status_updated):
            time.sleep(0.01)
        print 'Done'

        self.reset_arm_slider_to_current_value()
        self.reset_gripper_position_slider_to_current_value()

        print 'Getting', self.name, 'current control mode ...',
        sys.stdout.flush()
        get_current_control_mode = rospy.ServiceProxy('/' + self.name + '/get_control_mode_service', GetControlMode)
        control_mode = get_current_control_mode()
        self.control_mode_combobox.setCurrentIndex(control_mode.active_control_mode.control_mode.mode)
        print 'Done'

    def arm_status_callback(self, data):
        self.arm_status = data
        self.arm_status_updated = True

    def gripper_status_callback(self, data):
        self.gripper_status = data
        self.gripper_status_updated = True

    def skip_rows(self, col, num_rows):
        self.row_count[col] = self.row_count[col] + num_rows

    def init_button(self, text, col, slot_function, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        button = QPushButton(text)
        self.v_layout.addWidget(button, self.row_count[col], col)
        button.clicked.connect(slot_function)
        self.row_count[col] += 1

        return button

    def init_slider(self, range, col, slot_function, init_position=None, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        slider = QSlider()
        slider.setOrientation(Qt.Horizontal)
        slider.setRange(range[0], range[1])
        if init_position is not None:
            slider.setValue(init_position)
        slider.valueChanged.connect(slot_function)

        self.v_layout.addWidget(slider, self.row_count[col], col)
        self.row_count[col] += 1

        return slider

    def init_label(self, text, col, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        label = QLabel(text)
        self.v_layout.addWidget(label, self.row_count[col], col)
        self.row_count[col] += 1

        return label

    def init_textbox(self, text, col, slot_function, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        textbox = QLineEdit(text)
        self.v_layout.addWidget(textbox, self.row_count[col], col)
        textbox.editingFinished.connect(slot_function)
        self.row_count[col] += 1

        return textbox

    def init_checkbox(self, text, col, slot_function, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        checkbox = QCheckBox(text)
        self.v_layout.addWidget(checkbox, self.row_count[col], col)
        checkbox.stateChanged.connect(slot_function)
        self.row_count[col] += 1

        return checkbox

    def init_combobox(self, option_texts, col, slot_function, num_rows_skipped=0):
        self.row_count[col] += num_rows_skipped
        combobox = QComboBox()
        for text in option_texts:
            combobox.addItem(text)
        self.v_layout.addWidget(combobox, self.row_count[col], col)
        combobox.currentIndexChanged.connect(slot_function)
        self.row_count[col] += 1

        return combobox

    def finger_same_position_checkbox_changed(self):
        if self.finger_same_position_checkbox.isChecked():
            self.fingers_same_position = True
        else:
            self.fingers_same_position = False

    def finger_same_speed_checkbox_changed(self):
        if self.finger_same_speed_checkbox.isChecked():
            self.fingers_same_speed = True
        else:
            self.fingers_same_speed = False

    def finger_same_force_checkbox_changed(self):
        if self.finger_same_force_checkbox.isChecked():
            self.fingers_same_force = True
        else:
            self.fingers_same_force = False

    def open_gripper_command(self):
        self.finger_a_pos_slider.setValue(0)
        self.finger_b_pos_slider.setValue(0)
        self.finger_c_pos_slider.setValue(0)
        self.finger_command.finger_a_command.position = 0
        self.finger_command.finger_b_command.position = 0
        self.finger_command.finger_c_command.position = 0
        self.finger_command_publisher.publish(self.finger_command)

    def close_gripper_command(self):
        self.finger_a_pos_slider.setValue(finger_range_discretization)
        self.finger_b_pos_slider.setValue(finger_range_discretization)
        self.finger_c_pos_slider.setValue(finger_range_discretization)
        self.finger_command.finger_a_command.position = 1
        self.finger_command.finger_b_command.position = 1
        self.finger_command.finger_c_command.position = 1
        self.finger_command_publisher.publish(self.finger_command)

    def reset_arm_command_to_zero(self):
        for joint in joint_names:
            self.joint_sliders[joint].setValue(0)

        self.arm_command.joint_position.joint_1 = 0
        self.arm_command.joint_position.joint_2 = 0
        self.arm_command.joint_position.joint_3 = 0
        self.arm_command.joint_position.joint_4 = 0
        self.arm_command.joint_position.joint_5 = 0
        self.arm_command.joint_position.joint_6 = 0
        self.arm_command.joint_position.joint_7 = 0

        self.arm_command.joint_velocity.joint_1 = 0
        self.arm_command.joint_velocity.joint_2 = 0
        self.arm_command.joint_velocity.joint_3 = 0
        self.arm_command.joint_velocity.joint_4 = 0
        self.arm_command.joint_velocity.joint_5 = 0
        self.arm_command.joint_velocity.joint_6 = 0
        self.arm_command.joint_velocity.joint_7 = 0

        self.arm_command_publisher.publish(self.arm_command)

    def reset_arm_slider_to_current_value(self):
        local_arm_status = copy.deepcopy(self.arm_status)

        global block_command
        block_command = True

        meas_pos = local_arm_status.measured_joint_position

        for joint in joint_names:
            pos_rad = getattr(meas_pos, joint)
            pos_deg = round(clip(math.degrees(pos_rad), joint))
            self.joint_sliders[joint].setValue(pos_deg)

        block_command = False

        self.arm_command.joint_position.joint_1 = local_arm_status.measured_joint_position.joint_1
        self.arm_command.joint_position.joint_2 = local_arm_status.measured_joint_position.joint_2
        self.arm_command.joint_position.joint_3 = local_arm_status.measured_joint_position.joint_3
        self.arm_command.joint_position.joint_4 = local_arm_status.measured_joint_position.joint_4
        self.arm_command.joint_position.joint_5 = local_arm_status.measured_joint_position.joint_5
        self.arm_command.joint_position.joint_6 = local_arm_status.measured_joint_position.joint_6
        self.arm_command.joint_position.joint_7 = local_arm_status.measured_joint_position.joint_7

        self.arm_command.joint_velocity.joint_1 = local_arm_status.measured_joint_velocity.joint_1
        self.arm_command.joint_velocity.joint_2 = local_arm_status.measured_joint_velocity.joint_2
        self.arm_command.joint_velocity.joint_3 = local_arm_status.measured_joint_velocity.joint_3
        self.arm_command.joint_velocity.joint_4 = local_arm_status.measured_joint_velocity.joint_4
        self.arm_command.joint_velocity.joint_5 = local_arm_status.measured_joint_velocity.joint_5
        self.arm_command.joint_velocity.joint_6 = local_arm_status.measured_joint_velocity.joint_6
        self.arm_command.joint_velocity.joint_7 = local_arm_status.measured_joint_velocity.joint_7

    def disable_arm_sliders(self):
        for joint in joint_names:
            self.joint_sliders[joint].setEnabled(False)
            self.joint_textboxes[joint].setReadOnly(True)

    def enable_arm_sliders(self):
        for joint in joint_names:
            self.joint_sliders[joint].setEnabled(True)
            self.joint_textboxes[joint].setReadOnly(False)

    def reset_speed_force_command(self):
        self.finger_a_spe_slider.setValue(finger_range_discretization)
        self.finger_b_spe_slider.setValue(finger_range_discretization)
        self.finger_c_spe_slider.setValue(finger_range_discretization)
        self.scissor_spe_slider.setValue(finger_range_discretization)
        self.finger_a_frc_slider.setValue(finger_range_discretization)
        self.finger_b_frc_slider.setValue(finger_range_discretization)
        self.finger_c_frc_slider.setValue(finger_range_discretization)
        self.scissor_frc_slider.setValue(finger_range_discretization)

        self.finger_command.finger_a_command.speed = 1
        self.finger_command.finger_b_command.speed = 1
        self.finger_command.finger_c_command.speed = 1
        self.finger_command.scissor_command.speed = 1
        self.finger_command.finger_a_command.force = 1
        self.finger_command.finger_b_command.force = 1
        self.finger_command.finger_c_command.force = 1
        self.finger_command.scissor_command.force = 1
        self.finger_command_publisher.publish(self.finger_command)

    def reset_gripper_position_slider_to_current_value(self):
        local_gripper_status = copy.deepcopy(self.gripper_status)

        global block_command

        block_command = True
        
        self.finger_a_pos_slider.setValue(int(round(local_gripper_status.finger_a_status.position * finger_range_discretization)))
        self.finger_b_pos_slider.setValue(int(round(local_gripper_status.finger_b_status.position * finger_range_discretization)))
        self.finger_c_pos_slider.setValue(int(round(local_gripper_status.finger_c_status.position * finger_range_discretization)))
        self.scissor_pos_slider.setValue(int(round(local_gripper_status.scissor_status.position * finger_range_discretization)))

        # self.finger_a_pos_textbox.setText('%5.3f' % round(local_gripper_status.finger_a_status.position,3))
        # self.finger_b_pos_textbox.setText('%5.3f' % round(local_gripper_status.finger_b_status.position,3))
        # self.finger_c_pos_textbox.setText('%5.3f' % round(local_gripper_status.finger_c_status.position,3))
        # self.scissor_pos_textbox.setText('%5.3f' % round(local_gripper_status.scissor_status.position,3))

        block_command = False

        self.finger_command.finger_a_command.position = local_gripper_status.finger_a_status.position
        self.finger_command.finger_b_command.position = local_gripper_status.finger_b_status.position
        self.finger_command.finger_c_command.position = local_gripper_status.finger_c_status.position
        self.finger_command.scissor_command.position = local_gripper_status.scissor_status.position

    def move_all_fingers_pos(self, position):
        value = float(position) / finger_range_discretization
        value = min(max(value, 0), 1)
        position = int(value * finger_range_discretization)
        self.finger_a_pos_textbox.setText('%5.3f' % value)
        self.finger_command.finger_a_command.position = value
        self.finger_a_pos_slider.setValue(position)

        self.finger_b_pos_textbox.setText('%5.3f' % value)
        self.finger_command.finger_b_command.position = value
        self.finger_b_pos_slider.setValue(position)

        self.finger_c_pos_textbox.setText('%5.3f' % value)
        self.finger_command.finger_c_command.position = value
        self.finger_c_pos_slider.setValue(position)

        if not block_command:
            self.finger_command_publisher.publish(self.finger_command)

    def move_all_fingers_spe(self, position):
        value = float(position) / finger_range_discretization
        value = min(max(value, 0), 1)
        position = int(value * finger_range_discretization)
        self.finger_a_spe_textbox.setText('%5.3f' % value)
        self.finger_command.finger_a_command.speed = value
        self.finger_a_spe_slider.setValue(position)

        self.finger_b_spe_textbox.setText('%5.3f' % value)
        self.finger_command.finger_b_command.speed = value
        self.finger_b_spe_slider.setValue(position)

        self.finger_c_spe_textbox.setText('%5.3f' % value)
        self.finger_command.finger_c_command.speed = value
        self.finger_c_spe_slider.setValue(position)

        if not block_command:
            self.finger_command_publisher.publish(self.finger_command)

    def move_all_fingers_frc(self, position):
        value = float(position) / finger_range_discretization
        value = min(max(value, 0), 1)
        position = int(value * finger_range_discretization)
        self.finger_a_frc_textbox.setText('%5.3f' % value)
        self.finger_command.finger_a_command.force = value
        self.finger_a_frc_slider.setValue(position)

        self.finger_b_frc_textbox.setText('%5.3f' % value)
        self.finger_command.finger_b_command.force = value
        self.finger_b_frc_slider.setValue(position)

        self.finger_c_frc_textbox.setText('%5.3f' % value)
        self.finger_command.finger_c_command.force = value
        self.finger_c_frc_slider.setValue(position)

        if not block_command:
            self.finger_command_publisher.publish(self.finger_command)

    def finger_a_pos_textbox_modified(self):
        self.finger_a_pos_slider_moved(
            int(float(self.finger_a_pos_textbox.displayText()) * finger_range_discretization))

    def finger_a_spe_textbox_modified(self):
        self.finger_a_spe_slider_moved(
            int(float(self.finger_a_spe_textbox.displayText()) * finger_range_discretization))

    def finger_a_frc_textbox_modified(self):
        self.finger_a_frc_slider_moved(
            int(float(self.finger_a_frc_textbox.displayText()) * finger_range_discretization))

    def finger_b_pos_textbox_modified(self):
        self.finger_b_pos_slider_moved(
            int(float(self.finger_b_pos_textbox.displayText()) * finger_range_discretization))

    def finger_b_spe_textbox_modified(self):
        self.finger_b_spe_slider_moved(
            int(float(self.finger_b_spe_textbox.displayText()) * finger_range_discretization))

    def finger_b_frc_textbox_modified(self):
        self.finger_b_frc_slider_moved(
            int(float(self.finger_b_frc_textbox.displayText()) * finger_range_discretization))

    def finger_c_pos_textbox_modified(self):
        self.finger_c_pos_slider_moved(
            int(float(self.finger_c_pos_textbox.displayText()) * finger_range_discretization))

    def finger_c_spe_textbox_modified(self):
        self.finger_c_spe_slider_moved(
            int(float(self.finger_c_spe_textbox.displayText()) * finger_range_discretization))

    def finger_c_frc_textbox_modified(self):
        self.finger_c_frc_slider_moved(
            int(float(self.finger_c_frc_textbox.displayText()) * finger_range_discretization))

    def scissor_pos_textbox_modified(self):
        self.scissor_pos_slider_moved(int(float(self.scissor_pos_textbox.displayText()) * finger_range_discretization))

    def scissor_spe_textbox_modified(self):
        self.scissor_spe_slider_moved(int(float(self.scissor_spe_textbox.displayText()) * finger_range_discretization))

    def scissor_frc_textbox_modified(self):
        self.scissor_frc_slider_moved(int(float(self.scissor_frc_textbox.displayText()) * finger_range_discretization))

    def finger_a_pos_slider_moved(self, position):
        if self.fingers_same_position:
            self.move_all_fingers_pos(position)
        else:
            value = float(position) / finger_range_discretization
            value = min(max(value, 0), 1)
            position = int(value * finger_range_discretization)
            self.finger_a_pos_textbox.setText('%5.3f' % value)
            self.finger_a_pos_slider.setValue(position)
            self.finger_command.finger_a_command.position = value
            if not block_command:
                self.finger_command_publisher.publish(self.finger_command)

    def finger_a_spe_slider_moved(self, position):
        if self.fingers_same_speed:
            self.move_all_fingers_spe(position)
        else:
            value = float(position) / finger_range_discretization
            value = min(max(value, 0), 1)
            position = int(value * finger_range_discretization)
            self.finger_a_spe_textbox.setText('%5.3f' % value)
            self.finger_a_spe_slider.setValue(position)
            self.finger_command.finger_a_command.speed = value
            if not block_command:
                self.finger_command_publisher.publish(self.finger_command)

    def finger_a_frc_slider_moved(self, position):
        if self.fingers_same_force:
            self.move_all_fingers_frc(position)
        else:
            value = float(position) / finger_range_discretization
            value = min(max(value, 0), 1)
            position = int(value * finger_range_discretization)
            self.finger_a_frc_textbox.setText('%5.3f' % value)
            self.finger_a_frc_slider.setValue(position)
            self.finger_command.finger_a_command.force = value
            if not block_command:
                self.finger_command_publisher.publish(self.finger_command)

    def finger_b_pos_slider_moved(self, position):
        if self.fingers_same_position:
            self.move_all_fingers_pos(position)
        else:
            value = float(position) / finger_range_discretization
            value = min(max(value, 0), 1)
            position = int(value * finger_range_discretization)
            self.finger_b_pos_textbox.setText('%5.3f' % value)
            self.finger_b_pos_slider.setValue(position)
            self.finger_command.finger_b_command.position = value
            if not block_command:
                self.finger_command_publisher.publish(self.finger_command)

    def finger_b_spe_slider_moved(self, position):
        if self.fingers_same_speed:
            self.move_all_fingers_spe(position)
        else:
            value = float(position) / finger_range_discretization
            value = min(max(value, 0), 1)
            position = int(value * finger_range_discretization)
            self.finger_b_spe_textbox.setText('%5.3f' % value)
            self.finger_b_spe_slider.setValue(position)
            self.finger_command.finger_b_command.speed = value
            if not block_command:
                self.finger_command_publisher.publish(self.finger_command)

    def finger_b_frc_slider_moved(self, position):
        if self.fingers_same_force:
            self.move_all_fingers_frc(position)
        else:
            value = float(position) / finger_range_discretization
            value = min(max(value, 0), 1)
            position = int(value * finger_range_discretization)
            self.finger_b_frc_textbox.setText('%5.3f' % value)
            self.finger_b_frc_slider.setValue(position)
            self.finger_command.finger_b_command.force = value
            if not block_command:
                self.finger_command_publisher.publish(self.finger_command)

    def finger_c_pos_slider_moved(self, position):
        if self.fingers_same_position:
            self.move_all_fingers_pos(position)
        else:
            value = float(position) / finger_range_discretization
            value = min(max(value, 0), 1)
            position = int(value * finger_range_discretization)
            self.finger_c_pos_textbox.setText('%5.3f' % value)
            self.finger_c_pos_slider.setValue(position)
            self.finger_command.finger_c_command.position = value
            if not block_command:
                self.finger_command_publisher.publish(self.finger_command)

    def finger_c_spe_slider_moved(self, position):
        if self.fingers_same_speed:
            self.move_all_fingers_spe(position)
        else:
            value = float(position) / finger_range_discretization
            value = min(max(value, 0), 1)
            position = int(value * finger_range_discretization)
            self.finger_c_spe_textbox.setText('%5.3f' % value)
            self.finger_c_spe_slider.setValue(position)
            self.finger_command.finger_c_command.speed = value
            if not block_command:
                self.finger_command_publisher.publish(self.finger_command)

    def finger_c_frc_slider_moved(self, position):
        if self.fingers_same_force:
            self.move_all_fingers_frc(position)
        else:
            value = float(position) / finger_range_discretization
            value = min(max(value, 0), 1)
            position = int(value * finger_range_discretization)
            self.finger_c_frc_textbox.setText('%5.3f' % value)
            self.finger_c_frc_slider.setValue(position)
            self.finger_command.finger_c_command.force = value
            if not block_command:
                self.finger_command_publisher.publish(self.finger_command)

    def scissor_pos_slider_moved(self, position):
        value = float(position) / finger_range_discretization
        value = min(max(value, 0), 1)
        position = int(value * finger_range_discretization)
        self.scissor_pos_textbox.setText('%5.3f' % value)
        self.scissor_pos_slider.setValue(position)
        self.finger_command.scissor_command.position = value
        if not block_command:
            self.finger_command_publisher.publish(self.finger_command)

    def scissor_spe_slider_moved(self, position):
        value = float(position) / finger_range_discretization
        value = min(max(value, 0), 1)
        position = int(value * finger_range_discretization)
        self.scissor_spe_textbox.setText('%5.3f' % value)
        self.scissor_spe_slider.setValue(position)
        self.finger_command.scissor_command.speed = value
        if not block_command:
            self.finger_command_publisher.publish(self.finger_command)

    def scissor_frc_slider_moved(self, position):
        value = float(position) / finger_range_discretization
        value = min(max(value, 0), 1)
        position = int(value * finger_range_discretization)
        self.scissor_frc_textbox.setText('%5.3f' % value)
        self.scissor_frc_slider.setValue(position)
        self.finger_command.scissor_command.force = value
        if not block_command:
            self.finger_command_publisher.publish(self.finger_command)


    def joint_textbox_modified(self, joint_name):
        value = int(self.joint_textboxes[joint_name].displayText())
        value = clip(value, joint_name)
        self.joint_slider_moved(value, joint_name)


    def joint_slider_moved(self, position, joint_name):
        self.joint_textboxes[joint_name].setText(str(position))
        self.joint_sliders[joint_name].setValue(position)
        setattr(self.arm_command.joint_position, joint_name, math.radians(position))
        if not block_command:
            self.arm_command_publisher.publish(self.arm_command)


    def change_control_mode(self, control_mode):
        # rospy.wait_for_service('set_control_mode_service')
        send_new_control_mode = rospy.ServiceProxy('/' + self.name + '/set_control_mode_service', SetControlMode)
        new_control_mode = ControlModeParameters()
        new_control_mode.control_mode.mode = control_mode

        if control_mode == ControlMode.JOINT_POSITION:
            new_control_mode.joint_path_execution_params.joint_relative_velocity = 0.1
            new_control_mode.joint_path_execution_params.joint_relative_acceleration = 0.1
            self.enable_arm_sliders()
            print('Switching to JOINT_POSITION mode.')
        elif control_mode == ControlMode.CARTESIAN_POSE:
            self.disable_arm_sliders()
            print('Switching to CARTESIAN_POSE mode.')
        elif control_mode == ControlMode.JOINT_IMPEDANCE:
            print('JOINT_IMPEDANCE mode switch is not implemented yet.')
            return
        elif control_mode == ControlMode.CARTESIAN_IMPEDANCE:
            print('CARTESIAN_IMPEDANCE mode switch is not implemented yet.')
            return

        result = send_new_control_mode(new_control_mode)
        if result.success:
            print "Control mode switch success."
        else:
            print "Control mode switch failure."
            print result.message


def main():
    rospy.init_node('arm_command_widget')
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)

    widget = Widget()
    widget.setWindowTitle("Arm Command Widget")
    widget.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
o
