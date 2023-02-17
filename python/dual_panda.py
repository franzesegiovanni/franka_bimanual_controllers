"""
Authors: Giovanni Franzese and Tim Verburg, June 2022
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
# !/usr/bin/env python
import os
import queue
import matplotlib
import rospy
import math
import numpy as np
import quaternion
import time
import pandas as pd
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Point, WrenchStamped, PoseStamped, Vector3
import dynamic_reconfigure.client
from std_msgs.msg import Float32MultiArray, Bool, Float32
import pathlib
from pynput.keyboard import Listener, KeyCode
from franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal


class DualPanda:
    def __init__(self, arm_id_right='panda_right', arm_id_left='panda_left'):
        rospy.init_node('DualArmControl', anonymous=True)
        self.client = dynamic_reconfigure.client.Client("dual_teaching", timeout=None, config_callback=None)
        
        self.control_freq = 50
        self.rec_freq = 50

        # The execution factor is a multiplier used to accelerate or slow down an execution
        self.execution_factor = 1
        self.client.update_configuration(self.client.update_configuration({"execution_factor": self.execution_factor}))

        # Factors that multiply the feedback from the Spacemouse to allow for more rough or granular control
        self.feedback_factor_pos = 0.02
        self.feedback_factor_ori = 0.02
        self.feedback_factor_stiffness = 0.05

        self.target_coupling_stiffness = 800

        self.Panda_right = Panda(self.rec_freq, self.control_freq, self.feedback_factor_pos, self.feedback_factor_stiffness, arm_id_right)
        self.Panda_left = Panda(self.rec_freq, self.control_freq, self.feedback_factor_pos, self.feedback_factor_stiffness, arm_id_left)
        
        rospy.Subscriber("/spacenav_left/offset", Vector3, self.teleop_callback_left, queue_size=1)
        rospy.Subscriber("/spacenav_left/joy", Joy, self.btns_callback_left, queue_size=1)
        rospy.Subscriber("/spacenav_right/offset", Vector3, self.teleop_callback_right, queue_size=1)
        rospy.Subscriber("/spacenav_right/joy", Joy, self.btns_callback_right, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.coupling_diff_pub = rospy.Publisher("/panda_dual/bimanual_cartesian_impedance_controller/equilibrium_distance", PoseStamped, queue_size=0)
        
        # Start keyboard listener
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

        # enable_correction is used to enable/disable corrections using the spacemouse
        self.enable_correction = False

        self.external_record = False

        # The index variable is updated to allow for easy access to the current index position within the execution trajectory
        self.index = 0

        self.end = True

    def _on_press(self, key):
        """
        Function that runs in the background and checks for a key press
        """
        if key == KeyCode.from_char('e'):
            self.end = True
            self.Panda_left.end = True
            self.Panda_right.end = True

        if key == KeyCode.from_char('u') and (self.Panda_left.recording or not self.Panda_left.completed):
            self.Panda_left.pause = not self.Panda_left.pause
            print(f"Panda left pause toggled, state: {self.Panda_left.pause}")

        if key == KeyCode.from_char('i') and (self.Panda_right.recording or not self.Panda_right.completed):
            self.Panda_right.pause = not self.Panda_right.pause
            print(f"Panda right toggled, state: {self.Panda_right.pause}")

    def joy_callback(self, data):
        """"
        Callback function for the joystick, written for the Logitech F710
        """
        # B
        if data.buttons[2] == 1:
            self.end = True
            self.Panda_left.end = True
            self.Panda_right.end = True
        # LB
        if data.buttons[4] == 1:
            self.Panda_right.pause = not self.Panda_right.pause
            print("Panda right toggled, state: {self.Panda_left.pause}")

        # RB
        if data.buttons[5] == 1:
            self.Panda_left.pause = not self.Panda_left.pause
            print("Panda left pause toggled, state: {self.Panda_left.pause}")

        # Dpad up/down
        if data.buttons[6] == 1:
            self.execution_factor += 0.2
            self.client.update_configuration({"execution_factor": self.execution_factor})
        if data.buttons[7] == 1:
            self.execution_factor -= 0.2
            self.client.update_configuration({"execution_factor": self.execution_factor})


    def teleop_callback_left(self, data):
        """
        Spacemouse callback for the left Panda
        :param data: callback data
        """
        self.Panda_left.feedback =[data.x, data.y, data.z]

        if self.Panda_left.completed == False and self.Panda_left.enable_corr and np.sum(self.Panda_left.feedback) != 0:
            if self.Panda_left.correction_mode == 0:
                self.Panda_left.execution_traj = self.Panda_left.correct_attractor(self.Panda_left.index, self.Panda_left.execution_traj, self.Panda_left.feedback_factor_pos)
                time.sleep(0.01)
            else:
                self.Panda_left.execution_stiff_lin = self.Panda_left.correct_stiffness(self.Panda_left.index, self.Panda_left.execution_traj, self.Panda_left.execution_stiff_lin, self.Panda_left.feedback_factor_stiff_lin)
                time.sleep(0.01)

    def btns_callback_left(self, data):
        """
        Spacemouse button callback for the left Panda
        :param data: callback data
        """
        self.Panda_left.btn = [data.buttons[0], data.buttons[1]]

    def teleop_callback_right(self, data):
        """
        Spacemouse callback for the right Panda
        :param data: callback data
        """
        self.Panda_right.feedback = [data.x, data.y, data.z]

        if self.Panda_right.completed == False and self.Panda_right.enable_corr and np.sum(self.Panda_right.feedback) != 0:
            if self.Panda_right.correction_mode == 0:
                self.Panda_right.execution_traj = self.Panda_right.correct_attractor(self.Panda_right.index, self.Panda_right.execution_traj, self.Panda_right.feedback_factor_pos)
                time.sleep(0.01)
            else:
                self.Panda_right.execution_stiff_lin = self.Panda_right.correct_stiffness(self.Panda_right.index, self.Panda_right.execution_traj, self.Panda_right.execution_stiff_lin, self.Panda_right.feedback_factor_stiff_lin)
                time.sleep(0.01)

    # spacemouse buttons subscriber
    def btns_callback_right(self, data):
        """
        Spacemouse button callback for the left Panda
        :param data: callback data
        """
        self.Panda_right.btn = [data.buttons[0], data.buttons[1]]

    def Kinesthetic_Demonstration_BiManual(self, trigger=0.005, active=False, verbose=True):
        """
        Funtion to record a bimanual demonstration.

        :param trigger: The velocity threshold for which the demonstration start is detected
        :param active: A boolean
        that determines the stiffness of the arms. If you have Active equal to true, you are recording but the
        manipulators' stiffness is not dropped to zero. You can for example learn to synchronize what two
        manipulators are doing
        :param verbose: Boolean that determines if the function prints to the terminal
        """
        self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")

        self.Panda_left.update_params()
        self.Panda_right.update_params()
    
        r = rospy.Rate(self.control_freq*self.execution_factor)
        
        if active is False:
            self.Panda_left.Passive()
            self.Panda_right.Passive()
        else:
            self.Panda_left.Active()
            self.Panda_right.Active()

        self.end = False

        init_pos_right = self.Panda_right.cart_pos
        init_pos_left = self.Panda_left.cart_pos

        vel_right = 0
        vel_left = 0


        if verbose:
            print("Move robot to start recording.")

        while vel_right < trigger and vel_left < trigger:
            vel_right = math.sqrt((self.Panda_right.cart_pos[0] - init_pos_right[0]) ** 2 + (
                    self.Panda_right.cart_pos[1] - init_pos_right[1]) ** 2 + (
                                          self.Panda_right.cart_pos[2] - init_pos_right[2]) ** 2)
            vel_left = math.sqrt((self.Panda_left.cart_pos[0] - init_pos_left[0]) ** 2 + (
                    self.Panda_left.cart_pos[1] - init_pos_left[1]) ** 2 + (
                                         self.Panda_left.cart_pos[2] - init_pos_left[2]) ** 2)
        if verbose:
            print("Recording started. Press e to stop.")

        if self.external_record:
                rospy.set_param("/dual_teaching/recording", True)

        self.recorded_traj_dual = np.r_[self.Panda_right.cart_pos, self.Panda_left.cart_pos]
        self.recorded_joint_dual = np.r_[self.Panda_right.joint_pos, self.Panda_left.joint_pos]
        self.recorded_ori_dual = np.r_[self.Panda_right.cart_ori, self.Panda_left.cart_ori]
        self.recorded_gripper_dual = np.array([self.Panda_right.gripper_width, self.Panda_left.gripper_width])
        self.recorded_stiffness_lin_dual = np.array([self.Panda_right.K_cart, self.Panda_right.K_cart, self.Panda_right.K_cart, self.Panda_left.K_cart, self.Panda_left.K_cart, self.Panda_left.K_cart])
        self.recorded_stiffness_ori_dual = np.array([self.Panda_right.K_ori, self.Panda_right.K_ori, self.Panda_right.K_ori, self.Panda_left.K_ori, self.Panda_left.K_ori, self.Panda_left.K_ori])

        while (not self.end) and (not self.enable_correction*self.Panda_left.completed*self.Panda_right.completed):
            self.recorded_traj_dual = np.c_[
                self.recorded_traj_dual, np.r_[self.Panda_right.cart_pos, self.Panda_left.cart_pos]]
            self.recorded_ori_dual = np.c_[
                self.recorded_ori_dual, np.r_[self.Panda_right.cart_ori, self.Panda_left.cart_ori]]
            self.recorded_joint_dual = np.c_[
                self.recorded_joint_dual, np.r_[self.Panda_right.joint_pos, self.Panda_left.joint_pos]]
            self.recorded_gripper_dual = np.c_[
                self.recorded_gripper_dual, np.array([self.Panda_right.gripper_width, self.Panda_left.gripper_width])]
            self.recorded_stiffness_lin_dual = np.c_[
                self.recorded_stiffness_lin_dual, np.array([self.Panda_right.K_cart, self.Panda_right.K_cart, self.Panda_right.K_cart, 
                self.Panda_left.K_cart, self.Panda_left.K_cart, self.Panda_left.K_cart])]
            self.recorded_stiffness_ori_dual = np.c_[
                self.recorded_stiffness_ori_dual, np.array([self.Panda_right.K_ori, self.Panda_right.K_ori, self.Panda_right.K_ori, 
                self.Panda_left.K_ori, self.Panda_left.K_ori, self.Panda_left.K_ori])]
            r.sleep()

        if self.external_record:
                rospy.set_param("/dual_teaching/recording", False)

    def execute_dual(self, record=False):
        """
        Function to execute a bimanual demonstration. This execution will pause if one of the arms is outside their
        respective attractor distance threshold and thus cannot be used for synchronisation.
        """
        self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")
        r = rospy.Rate(self.control_freq*self.execution_factor)

        self.enable_correction = rospy.get_param("/dual_teaching/dual_enable_correction")
        
        self.Panda_left.update_params()
        self.Panda_right.update_params()

        self.Panda_left.pause = False
        self.Panda_right.pause = False
           
        self.index = 0
        self.Panda_left.index = self.Panda_right.index = self.index

        # Load the recorded trajectories into the execution trajectory to allow for generalised functions
        self.Panda_right.execution_traj = self.recorded_traj_dual[:3]
        self.Panda_left.execution_traj = self.recorded_traj_dual[3:]
        
        self.Panda_right.execution_ori = self.recorded_ori_dual[:4]
        self.Panda_left.execution_ori = self.recorded_ori_dual[4:]

        position_right = [self.Panda_right.execution_traj[0][0], self.Panda_right.execution_traj[1][0], self.Panda_right.execution_traj[2][0]]
        orientation_right = [self.Panda_right.execution_ori[0][0], self.Panda_right.execution_ori[1][0], self.Panda_right.execution_ori[2][0],
                           self.Panda_right.execution_ori[3][0]]

        position_left = [self.Panda_left.execution_traj[0][0], self.Panda_left.execution_traj[1][0], self.Panda_left.execution_traj[2][0]]
        orientation_left = [self.Panda_left.execution_ori[0][0], self.Panda_left.execution_ori[1][0], self.Panda_left.execution_ori[2][0],
                               self.Panda_left.execution_ori[3][0]]
        
        if self.Panda_left.start_safety_check(position_left) == False or self.Panda_right.start_safety_check(position_right) == False:
            return

        # TODO Verify if this is wanted in the code as it prevents dual execution with other stiffness params
        prefix = '/dynamic_reconfigure_compliance_param_node'

        self.Panda_left.set_stiffness(rospy.get_param(prefix + '/panda_left_translational_stiffness_X'), 
                                      rospy.get_param(prefix + '/panda_left_translational_stiffness_Y'), 
                                      rospy.get_param(prefix + '/panda_left_translational_stiffness_Z'), 
                                      rospy.get_param(prefix + '/panda_left_rotational_stiffness_X'), 
                                      rospy.get_param(prefix + '/panda_left_rotational_stiffness_Y'), 
                                      rospy.get_param(prefix + '/panda_left_rotational_stiffness_Z'), 
                                      rospy.get_param(prefix + '/panda_left_nullspace_stiffness'))

        self.Panda_right.set_stiffness(rospy.get_param(prefix + '/panda_right_translational_stiffness_X'), 
                                      rospy.get_param(prefix + '/panda_right_translational_stiffness_Y'), 
                                      rospy.get_param(prefix + '/panda_right_translational_stiffness_Z'), 
                                      rospy.get_param(prefix + '/panda_right_rotational_stiffness_X'), 
                                      rospy.get_param(prefix + '/panda_right_rotational_stiffness_Y'), 
                                      rospy.get_param(prefix + '/panda_right_rotational_stiffness_Z'), 
                                      rospy.get_param(prefix + '/panda_right_nullspace_stiffness'))

        # Code to calculate the difference between the arms and publish it to the coupling topic
        d_coupling = np.array(position_right) - np.array(position_left)
        diff_msg = PoseStamped()
        diff_msg.header.seq = 1
        diff_msg.header.stamp = rospy.Time.now()
        diff_msg.pose.position.x = d_coupling[0]
        diff_msg.pose.position.y = d_coupling[1]
        diff_msg.pose.position.z = d_coupling[2]        

        self.coupling_diff_pub.publish(diff_msg)

        rospy.get_param("/dual_teaching/target_coupling_stiffness")
        self.Panda_left.set_K.update_configuration({"coupling_translational_stiffness": self.target_coupling_stiffness})

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", True)

        while self.index < self.recorded_traj_dual.shape[1]:

            while self.Panda_left.pause or self.Panda_right.pause:
                r.sleep()

            self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")
            r = rospy.Rate(self.control_freq*self.execution_factor)
            if self.continue_dual_traj(self.index, self.recorded_traj_dual):

                position_right = [self.Panda_right.execution_traj[0][self.index], self.Panda_right.execution_traj[1][self.index], self.Panda_right.execution_traj[2][self.index]]
                orientation_right = [self.Panda_right.execution_ori[0][self.index], self.Panda_right.execution_ori[1][self.index], self.Panda_right.execution_ori[2][self.index],
                           self.Panda_right.execution_ori[3][self.index]]


                self.Panda_right.set_attractor(position_right, orientation_right)

                self.Panda_right.move_gripper(self.recorded_gripper_dual[0, self.index])

                position_left = [self.Panda_left.execution_traj[0][self.index], self.Panda_left.execution_traj[1][self.index], self.Panda_left.execution_traj[2][self.index]]
                orientation_left = [self.Panda_left.execution_ori[0][self.index], self.Panda_left.execution_ori[1][self.index], self.Panda_left.execution_ori[2][self.index],
                               self.Panda_left.execution_ori[3][self.index]]

                self.Panda_left.set_attractor(position_left, orientation_left)

                self.Panda_left.move_gripper(self.recorded_gripper_dual[1, self.index])

                # Code to calculate the difference between the arms and publish it to the coupling topic
                d_coupling = np.array(position_right) - np.array(position_left)
                diff_msg = PoseStamped()
                diff_msg.header.seq = 1
                diff_msg.header.stamp = rospy.Time.now()
                diff_msg.pose.position.x = d_coupling[0]
                diff_msg.pose.position.y = d_coupling[1]
                diff_msg.pose.position.z = d_coupling[2]
                
                self.coupling_diff_pub.publish(diff_msg)

                self.index += 1
                self.Panda_left.index = self.Panda_right.index = self.index

            r.sleep()

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", False)

        # Set the coupling stiffness back to 0
        self.Panda_left.set_K.update_configuration({"coupling_translational_stiffness": 0})

        
        # Overwrite the recorded trajectories with the execution trajectory as that includes the corrections done during execution
        self.recorded_traj_dual[:3] = self.Panda_right.execution_traj
        self.recorded_traj_dual[3:] = self.Panda_left.execution_traj

        self.recorded_ori_dual[:4] = self.Panda_right.execution_ori
        self.recorded_ori_dual[4:] = self.Panda_left.execution_ori

    def correction_execute_dual(self):
        """
        Function that allows for the execution of a bimanual demonstration while also allowing for corrections from the
        Spacemouse and also kinesthetic corrections. As the trajectory is split and executed separately, it allows for
        synchronisation of the tasks.
        """
        self.Panda_right.recorded_traj = self.recorded_traj_dual[:3]
        self.Panda_right.recorded_ori = self.recorded_ori_dual[:4]
        self.Panda_right.recorded_gripper = np.c_[self.recorded_gripper_dual[0]].T
        self.Panda_right.recorded_stiffness_lin = self.recorded_stiffness_lin_dual[:3]
        self.Panda_right.recorded_stiffness_ori = self.recorded_stiffness_ori_dual[:3]

        self.Panda_left.recorded_traj = self.recorded_traj_dual[3:]
        self.Panda_left.recorded_ori = self.recorded_ori_dual[4:]
        self.Panda_left.recorded_gripper = np.c_[self.recorded_gripper_dual[1]].T
        self.Panda_left.recorded_stiffness_lin = self.recorded_stiffness_lin_dual[3:]
        self.Panda_left.recorded_stiffness_ori = self.recorded_stiffness_ori_dual[3:]

        self.client.update_configuration({"panda_left_enable_correction":True, "panda_right_enable_correction":True, "dual_enable_correction": True})

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", True)

        self.Panda_left.execute_traj()
        self.Panda_right.execute_traj()
        self.Kinesthetic_Demonstration_BiManual(active=True, verbose=True)

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", False)

        self.client.update_configuration({"panda_left_enable_correction":False, "panda_right_enable_correction":False, "dual_enable_correction": False})

    def go_to_start(self):
        """
        Function used to set both arms to the first position of the demonstrated trajectories.
        """
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.recorded_traj_dual[0][0]
        goal.pose.position.y = self.recorded_traj_dual[1][0]
        goal.pose.position.z = self.recorded_traj_dual[2][0]

        goal.pose.orientation.w = self.recorded_ori_dual[0][0]
        goal.pose.orientation.x = self.recorded_ori_dual[1][0]
        goal.pose.orientation.y = self.recorded_ori_dual[2][0]
        goal.pose.orientation.z = self.recorded_ori_dual[3][0]

        self.Panda_right.goto_pub.publish(goal)

        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.recorded_traj_dual[3][0]
        goal.pose.position.y = self.recorded_traj_dual[4][0]
        goal.pose.position.z = self.recorded_traj_dual[5][0]

        goal.pose.orientation.w = self.recorded_ori_dual[4][0]
        goal.pose.orientation.x = self.recorded_ori_dual[5][0]
        goal.pose.orientation.y = self.recorded_ori_dual[6][0]
        goal.pose.orientation.z = self.recorded_ori_dual[7][0]

        self.Panda_left.goto_pub.publish(goal)

    def save(self, data='last'):
        """
        Function to save the last bimanual demonstration to a file.
        """
        if not os.path.exists(str(pathlib.Path().resolve()) + '/data'):
            os.mkdir(str(pathlib.Path().resolve()) + '/data')

        np.savez(str(pathlib.Path().resolve()) + '/data/' + 'dual_' + str(data) + '.npz',
                 recorded_traj_dual=self.recorded_traj_dual,
                 recorded_ori_dual=self.recorded_ori_dual,
                 recorded_gripper_dual=self.recorded_gripper_dual,
                 recorded_stiffness_lin_dual =self.recorded_stiffness_lin_dual,
                 recorded_stiffness_ori_dual = self.recorded_stiffness_ori_dual)

    def load(self, data='last'):
        """
        Function to load a previously saved bimanual demonstration.
        """
        data = np.load(str(pathlib.Path().resolve()) + '/data/' + 'dual_' + str(data) + '.npz')

        self.recorded_traj_dual = data['recorded_traj_dual'],
        self.recorded_ori_dual = data['recorded_ori_dual'],
        self.recorded_gripper_dual = data['recorded_gripper_dual'],
        self.recorded_stiffness_lin_dual = data['recorded_stiffness_lin_dual'],
        self.recorded_stiffness_ori_dual = data['recorded_stiffness_ori_dual'],
        self.recorded_traj_dual = self.recorded_traj_dual[0]
        self.recorded_ori_dual = self.recorded_ori_dual[0]
        self.recorded_gripper_dual = self.recorded_gripper_dual[0]
        self.recorded_stiffness_lin_dual = self.recorded_stiffness_lin_dual[0]
        self.recorded_stiffness_ori_dual = self.recorded_stiffness_ori_dual[0]

    def continue_dual_traj(self, index, traj):
        """
        Function that is used to verify if either arm is outside the attractor distance threshold.
        :param index: The index of the location on the trajectory to verify
        :param traj: The trajectory to compare against
        """
        if not (self.Panda_left.continue_traj(index, traj[3:])) or not (
                self.Panda_right.continue_traj(index, traj[:3])):
            return False
        else:
            return True


class Panda:
    
    def __init__(self, rec_frequency,control_frequency, ff_pos, ff_stiff_lin, arm_id=''):
        # ff = feedback_factor
    
        self.rec_freq = rec_frequency
        self.control_freq = control_frequency

        # The execution factor is a multiplier used to accelerate or slow down an execution
        self.execution_factor = 1

        self.name = arm_id

        # Distance threshold of the current cart position and the first point of the trajectory.
        # If greater than the threshold the trajectory will not start
        self.start_safety_threshold = 0.05

        self.set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        self.K_ori = 30.0
        self.K_cart = 600.0
        self.K_null = 0.0

        self.start = True
        self.end = False

        self.grip_command = GraspActionGoal()
        self.home_command = HomingActionGoal()
        self.stop_command = StopActionGoal()

        # The index variable is updated to allow for easy access to the current index position within the execution trajectory
        self.index = 0

        # Execution variables used to for corrections
        self.execution_traj = np.zeros(3)
        self.execution_ori = np.zeros(3)
        self.execution_stiff_lin = np.zeros(3)
        self.execution_stiff_ori = np.zeros(3)

        self.gripper_width = 0
        self.grip_command.goal.epsilon.inner = 0.1
        self.grip_command.goal.epsilon.outer = 0.1
        self.grip_command.goal.speed = 0.1
        self.grip_command.goal.force = 5
        self.grip_command.goal.width = 1

        self.attractor_distance_threshold = 0.08
        self.trajectory_distance_threshold = 0.08

        self.length_scale = 0.05
        self.correction_window = 50
        self.correction_mode = 0

        self.feedback = np.zeros(3)
        self.btn = np.zeros(2)
        self.enable_corr = False
        self.completed = True
        self.pause = False
        self.recording = False
        self.external_record = False

        self.feedback_factor_pos=0.01
        self.feedback_factor_ori=0.001
        self.feedback_factor_stiff_lin=1
        self.feedback_factor_stiff_ori=0.1

        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_cartesian_pose",
                         PoseStamped, self.ee_pose_callback)
        rospy.Subscriber("panda_dual/" + str(self.name) + "_state_controller/joint_states", JointState,
                         self.joint_callback)
        rospy.Subscriber("/" + str(self.name) + "_franka_gripper/joint_states", JointState, self.gripper_callback)
        rospy.Subscriber("/" + str(self.name) + "/stiffness", Float32MultiArray, self.stiffness_callback, queue_size=1)

        self.goal_pub = rospy.Publisher(
            "/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_equilibrium_pose", PoseStamped,
            queue_size=0)
        self.configuration_pub = rospy.Publisher(
            "panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_nullspace", JointState,
            queue_size=0)
        # self.gripper_pub = rospy.Publisher(str(self.name)+ "_gripper",Float32, queue_size=0)
        self.gripper_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/grasp/goal", GraspActionGoal,
                                           queue_size=0)
        self.homing_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/homing/goal", HomingActionGoal,
                                          queue_size=0)
        self.stop_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/stop/goal", StopActionGoal,
                                          queue_size=0)

        self.stiffness_pub = rospy.Publisher("/" + str(self.name) + "/stiffness", Float32MultiArray,
                                          queue_size=0)

        self.nullspace_stiffness_pub = rospy.Publisher("/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_nullspace", JointState,
                                          queue_size=0)

        rospy.Subscriber("panda_dual/" + str(self.name) + "/goto", PoseStamped, self.go_to_3d)
        rospy.Subscriber("panda_dual/" + str(self.name) + "/execute", Bool, self.execute)

        self.goto_pub = rospy.Publisher("panda_dual/" + str(self.name) + "/goto", PoseStamped, queue_size=0)
        self.execute_pub = rospy.Publisher("panda_dual/" + str(self.name) + "/execute", Bool, queue_size=0)

        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True

    def ee_pose_callback(self, data):
        self.cart_pos = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.cart_ori = [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y,
                         data.pose.orientation.z]

    def stiffness_callback(self, stiffness):
        self.set_stiffness(stiffness.data[0],stiffness.data[1],stiffness.data[2],stiffness.data[3],stiffness.data[4],stiffness.data[5],stiffness.data[6])

    # joint angle subscriber
    def joint_callback(self, data):
        self.joint_pos = data.position[0:7]

    # gripper state subscriber
    def gripper_callback(self, data):
        self.gripper_width = np.copy(data.position[0] + data.position[1])

    def move_gripper(self, width):
        if width < 0.07 and self.grip_command.goal.width != 0:
            self.grip_command.goal.width = 0
            self.gripper_pub.publish(self.grip_command)

        elif width > 0.07 and self.grip_command.goal.width != 1:
            self.grip_command.goal.width = 1
            self.gripper_pub.publish(self.grip_command)

    def home_gripper(self):
        self.homing_pub.publish(self.home_command)

    def stop_gripper(self):
        self.stop_pub.publish(self.stop_command)


    def set_stiffness(self, k_t1, k_t2, k_t3, k_r1, k_r2, k_r3, k_ns):

        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_X": k_t1})
        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_Y": k_t2})
        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_Z": k_t3})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_X": k_r1})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_Y": k_r2})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_Z": k_r3})
        # self.set_K.update_configuration({str(self.name) + "_nullspace_stiffness": k_ns})

    def Active(self):
        self.set_stiffness(400.0, 400.0, 400.0, 30.0, 30.0, 30.0, 0.0)

    def Passive(self):
        self.set_stiffness(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def set_attractor(self, pos, quat):
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = pos[0]
        goal.pose.position.y = pos[1]
        goal.pose.position.z = pos[2]

        goal.pose.orientation.w = quat[0]
        goal.pose.orientation.x = quat[1]
        goal.pose.orientation.y = quat[2]
        goal.pose.orientation.z = quat[3]

        self.goal_pub.publish(goal)

    def set_configuration(self, joint):
        joint_des = Float32MultiArray()
        joint_des.data = np.array(joint).astype(np.float32)
        self.configuration_pub.publish(joint_des)

    def execute_traj(self):
        goal = Bool()
        goal.data = True
        self.execute_pub.publish(goal)

    def execute(self, start):
        self.update_params()

        self.pause = False

        if start.data is True:
            self.completed = False
            self.end = False
    
            self.execution_traj = np.asarray(self.recorded_traj)
            self.execution_ori = np.asarray(self.recorded_ori)
            self.execution_gripper = np.asarray(self.recorded_gripper)
            self.execution_stiff_lin = np.asarray(self.recorded_stiffness_lin)
            self.execution_stiff_ori = np.asarray(self.recorded_stiffness_ori)
            
            i = 0
            self.index = 0

            i_position = [self.execution_traj[0][0], self.execution_traj[1][0], self.execution_traj[2][0]]

            if self.start_safety_check(i_position) == False:
                return            
            
            self.set_stiffness(self.K_cart, self.K_cart, self.K_cart, self.K_ori, self.K_ori, self.K_ori, 0.0)

            if self.external_record:
                rospy.set_param("/dual_teaching/recording", True)

            while i < (self.execution_traj.shape[1]):
                self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")
                r = rospy.Rate(self.control_freq*self.execution_factor)

                while self.pause:
                    r.sleep()

                i = int(self.index)
                position = [self.execution_traj[0][i], self.execution_traj[1][i], self.execution_traj[2][i]]
                orientation = [self.execution_ori[0][i], self.execution_ori[1][i], self.execution_ori[2][i],
                               self.execution_ori[3][i]]

                stiff_msg = Float32MultiArray()
                stiff_msg.data = np.array([self.execution_stiff_lin[0][i], self.execution_stiff_lin[1][i], self.execution_stiff_lin[2][i], self.execution_stiff_ori[0][i],self.execution_stiff_ori[1][i],self.execution_stiff_ori[2][i], 0.0]).astype(np.float32)
                self.stiffness_pub.publish(stiff_msg)
                if self.continue_traj(i, self.execution_traj):
                    self.set_attractor(position, orientation)

                    self.move_gripper(self.execution_gripper[0, i])

                    if self.enable_corr and np.sum(self.feedback) != 0:
                        # self.execution_traj = self.correct_attractor(i, self.execution_traj, self.feedback_factor_pos)

                        # Correction mode flag, 0 for attractor position correction and 1 for stiffness correction
                        if self.correction_mode != 0:
                            self.correction_mode = 0

                    i = i + 1
                    self.index = i
                else:
                    # Logic to move the attractor if the distance to the trajectory exceeds a certain threshold
                    traj_error = np.linalg.norm(self.execution_traj.T - np.array(self.cart_pos), axis=1)
                    if min(traj_error) > self.trajectory_distance_threshold:

                        # Code to assign new attractor based on the index distance
                        idx = np.argpartition(traj_error, 10)[:10] # idx is the list of the 10 indices belonging to the 10 smallest euclidian distances
                        self.index = idx[np.argmin(np.abs(idx-self.index))]

                        # Code to assign new attractor only based on euc distance
                        # self.index = int(np.argmin(traj_error))            

                        i = self.index
                        position = [self.execution_traj[0][i], self.execution_traj[1][i], self.execution_traj[2][i]]
                        orientation = [self.execution_ori[0][i], self.execution_ori[1][i], self.execution_ori[2][i],
                               self.execution_ori[3][i]]

                        self.set_attractor(position, orientation)

                        self.move_gripper(self.execution_gripper[0, i])

                    if self.enable_corr and np.sum(self.feedback) != 0:
                        # Correction mode flag, 0 for attractor position correction and 1 for stiffness correction
                        if self.correction_mode != 1:
                            self.correction_mode = 1

                r.sleep()

            if self.external_record:
                rospy.set_param("/dual_teaching/recording", False)

            if not self.recording:
                self.recorded_traj = np.asarray(self.execution_traj)
                self.recorded_ori = np.asarray(self.execution_ori)
                self.recorded_gripper = np.asarray(self.execution_gripper)
                self.recorded_stiffness_lin = np.asarray(self.execution_stiff_lin)
                self.recorded_stiffness_ori = np.asarray(self.execution_stiff_ori)
        
        start.data = False
        self.completed = True

    def go_to_start(self):
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.recorded_traj[0][0]
        goal.pose.position.y = self.recorded_traj[1][0]
        goal.pose.position.z = self.recorded_traj[2][0]

        goal.pose.orientation.w = self.recorded_ori[0][0]
        goal.pose.orientation.x = self.recorded_ori[1][0]
        goal.pose.orientation.y = self.recorded_ori[2][0]
        goal.pose.orientation.z = self.recorded_ori[3][0]

        self.goto_pub.publish(goal)

    def go_to_3d(self, data):
        control_freq = 50
        start = self.cart_pos
        start_ori = self.cart_ori
        r = rospy.Rate(control_freq)
        # interpolate from start to goal with attractor distance of approx 1 mm
        goal_ = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        # goal_ori_ = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        q_start = np.quaternion(start_ori[0], start_ori[1], start_ori[2], start_ori[3])
        q_goal = np.quaternion(data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y,
                               data.pose.orientation.z)

        squared_dist = np.sum(np.subtract(start, goal_) ** 2, axis=0)
        dist = np.sqrt(squared_dist)
        interp_dist = 0.01  # [m]
        step_num = math.floor(dist / interp_dist)

        x = np.linspace(start[0], goal_[0], step_num)
        y = np.linspace(start[1], goal_[1], step_num)
        z = np.linspace(start[2], goal_[2], step_num)
        quat = np.slerp_vectorized(q_start, q_goal, 0)

        position = [x[0], y[0], z[0]]
        orientation = [quat.w, quat.x, quat.y, quat.z]

        self.set_attractor(position, orientation)
        self.set_stiffness(self.K_cart, self.K_cart, self.K_cart, self.K_ori, self.K_ori, self.K_ori, 0)

        for i in range(step_num):
            position = [x[i], y[i], z[i]]
            quat = np.slerp_vectorized(q_start, q_goal, i / step_num)
            orientation = [quat.w, quat.x, quat.y, quat.z]
            self.set_attractor(position, orientation)
            r.sleep()

    def Kinesthetic_Demonstration(self, trigger=0.02, active=False):
        self.update_params()
        self.recording = True
        self.pause = False
        time.sleep(1)
        r = rospy.Rate(self.control_freq*self.execution_factor)
        stiff_msg = Float32MultiArray()
        if not active:
            stiff_msg.data = np.array([0, 0, 0, 0, 0, 0, 0]).astype(np.float32)
            self.stiffness_pub.publish(stiff_msg)

        self.end = False
        init_pos = self.cart_pos
        vel = 0
        print("Move robot to start recording.")
        while vel < trigger:
            vel = math.sqrt((self.cart_pos[0] - init_pos[0]) ** 2 + (self.cart_pos[1] - init_pos[1]) ** 2 + (
                    self.cart_pos[2] - init_pos[2]) ** 2)

        print("Recording started. Press e to stop and press u to pause/continue.")

        self.recorded_traj = self.cart_pos
        self.recorded_ori = self.cart_ori
        self.recorded_joint = self.joint_pos
        self.recorded_gripper = self.gripper_width
        self.recorded_stiffness_lin=[self.K_cart, self.K_cart, self.K_cart]
        self.recorded_stiffness_ori=[self.K_ori, self.K_ori, self.K_ori]

        while not self.end:

            self.recorded_traj = np.c_[self.recorded_traj, self.cart_pos]
            self.recorded_ori = np.c_[self.recorded_ori, self.cart_ori]
            self.recorded_joint = np.c_[self.recorded_joint, self.joint_pos]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.gripper_width]
            self.recorded_stiffness_lin=np.c_[self.recorded_stiffness_lin, [self.K_cart, self.K_cart, self.K_cart]]
            self.recorded_stiffness_ori=np.c_[self.recorded_stiffness_ori, [self.K_ori, self.K_ori, self.K_ori]]

            if self.pause:
                self.set_attractor(self.cart_pos, self.cart_ori)

                self.move_gripper(self.recorded_gripper[0, -1])
            
                stiff_msg = Float32MultiArray()
                stiff_msg.data = np.array([600, 600, 600, 30, 30, 30, 0]).astype(np.float32)
                self.stiffness_pub.publish(stiff_msg)

                while self.pause:
                    if self.end:
                        break
                    r.sleep()   

                stiff_msg = Float32MultiArray()
                stiff_msg.data = np.array([0, 0, 0, 0, 0, 0, 0]).astype(np.float32)
                self.stiffness_pub.publish(stiff_msg)
                self.stop_gripper()

            r.sleep()
        
        self.recording = False

    def save(self, data='last'):
        if not os.path.exists(str(pathlib.Path().resolve()) + '/data'):
            os.mkdir(str(pathlib.Path().resolve()) + '/data')

        np.savez(str(pathlib.Path().resolve()) + '/data/' + str(self.name) + '_' + str(data) + '.npz',
                 recorded_traj=self.recorded_traj,
                 recorded_ori=self.recorded_ori,
                 recorded_gripper=self.recorded_gripper,
                 recorded_stiffness_lin=self.recorded_stiffness_lin,
                 recorded_stiffness_ori=self.recorded_stiffness_ori)

    def load(self, file='last'):
        data = np.load(str(pathlib.Path().resolve()) + '/data/' + str(self.name) + '_' + str(file) + '.npz')

        self.recorded_traj = data['recorded_traj'],
        self.recorded_ori = data['recorded_ori'],
        self.recorded_gripper = data['recorded_gripper'],
        self.recorded_stiffness_lin = data['recorded_stiffness_lin'],
        self.recorded_stiffness_ori = data['recorded_stiffness_ori'],
        self.recorded_traj = self.recorded_traj[0]
        self.recorded_ori = self.recorded_ori[0]
        self.recorded_gripper = self.recorded_gripper[0]
        self.recorded_stiffness_lin = self.recorded_stiffness_lin[0]
        self.recorded_stiffness_ori = self.recorded_stiffness_ori[0]

    def continue_traj(self, index, traj):
        if (np.linalg.norm(np.array(self.cart_pos)-traj[:, index])) <= self.attractor_distance_threshold:
            return True
        else:
            return False
     
    def square_exp(self, lengthscale, x_1, x_2):
        d = np.linalg.norm(x_1-x_2,2)
        return np.exp(-d ** 2 /(2*lengthscale ** 2))


    def correct_attractor(self, index, traj, magnitude):

        for j in range(int(max(0, index - self.correction_window)),
                    int(min(index + self.correction_window, traj.shape[1]-1))):
            delta_x = magnitude*self.feedback[0] * self.square_exp(self.length_scale, traj[:,j], traj[:,index])
            delta_y = magnitude*self.feedback[1] * self.square_exp(self.length_scale, traj[:,j], traj[:,index])
            delta_z = magnitude*self.feedback[2] * self.square_exp(self.length_scale, traj[:,j], traj[:,index])

            traj[0][j] += delta_x
            traj[1][j] += delta_y  
            traj[2][j] += delta_z
            
        self.feedback = np.zeros(3)
        return traj

    def correct_stiffness(self, index, traj, stiff,  magnitude):

        for j in range(int(max(0, index - self.correction_window)),
                       int(min(index + self.correction_window, traj.shape[1]-1))):
            delta_K_x = magnitude*np.abs(self.feedback[0]) * self.square_exp(self.length_scale, traj[:,j], traj[:,index])
            delta_K_y = magnitude*np.abs(self.feedback[1]) * self.square_exp(self.length_scale, traj[:,j], traj[:,index])
            delta_K_z = magnitude*np.abs(self.feedback[2]) * self.square_exp(self.length_scale, traj[:,j], traj[:,index])

            stiff[0][j] += delta_K_x
            stiff[1][j] += delta_K_y
            stiff[2][j] += delta_K_z

        self.feedback = np.zeros(3)
        return stiff


    def update_params(self):
        self.length_scale = rospy.get_param("/dual_teaching/correction_length_scale")
        self.correction_window = rospy.get_param("/dual_teaching/correction_window_size")
        self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")

        self.attractor_distance_threshold = rospy.get_param("/dual_teaching/attractor_distance_threshold")
        self.trajectory_distance_threshold = rospy.get_param("/dual_teaching/trajectory_distance_threshold")

        self.feedback_factor_pos = rospy.get_param(f"/dual_teaching/{self.name}_feedback_factor_position")
        self.enable_corr = rospy.get_param(f"/dual_teaching/{self.name}_enable_correction")

        # CODE TO DELETE AN ATTRACTOR, NOT ADAPTED YET
        # if self.feedback[3] > 0:
        #     self.recorded_traj = np.delete(self.recorded_traj, np.arange(np.min([i + 1, self.recorded_traj.shape[1]]),
        #                                                                  np.min([i + 6, self.recorded_traj.shape[1]])),
        #                                    1)  # this removes the ith column
        #     self.recorded_ori = np.delete(self.recorded_ori, np.arange(np.min([i + 1, self.recorded_ori.shape[1]]),
        #                                                                np.min([i + 6, self.recorded_ori.shape[1]])), 1)
        #     self.recorded_gripper = np.delete(self.recorded_gripper,
        #                                       np.arange(np.min([i + 1, self.recorded_ori.shape[1]]),
        #                                                 np.min([i + 6, self.recorded_ori.shape[1]])), 1)
    

    def start_safety_check(self, t_pos):
        if np.linalg.norm(np.array(self.cart_pos)-np.array(t_pos)) > self.start_safety_threshold:
            print(f"{self.name} is too far from the start position, please make sure the go_to_start function has been run")
            return False
        else:
            return True

    def home(self):
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = 0.6
        goal.pose.position.y = 0
        goal.pose.position.z = 0.4

        goal.pose.orientation.w = 0
        goal.pose.orientation.x = 1
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = 0

        

        ns_msg = JointState()   
        ns_msg.position = [0, 0, 0, -2.4, 0, 2.4, 0]

        self.goto_pub.publish(goal)
        self.nullspace_stiffness_pub.publish(ns_msg)
        self.set_K.update_configuration({f'{str(self.name)}_nullspace_stiffness':10})

        rospy.sleep(rospy.Duration(secs=5))

        self.set_K.update_configuration({f'{str(self.name)}_nullspace_stiffness':0})


# %%
