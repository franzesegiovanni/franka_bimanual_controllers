"""
Authors: Giovanni Franzese, June 2022
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
# !/usr/bin/env python
import rospy
import math
import numpy as np
import time
import quaternion 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client
from std_msgs.msg import Float32MultiArray, Bool
import pathlib
from pynput.keyboard import Listener, Key
from franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal
class Panda:
    
    def __init__(self, control_frequency=30, arm_id=''):
    
        self.control_freq = control_frequency

        self.name = arm_id

        # Distance threshold of the current cart position and the first point of the trajectory.
        # If greater than the threshold the trajectory will not start
        self.start_safety_threshold = 0.3

        self.set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        self.K_ori = 30.0
        self.K_cart = 400.0
        self.K_null = 0.0

        self.end = False

        self.grip_command = GraspActionGoal()
        self.home_command = HomingActionGoal()
        self.gripper_width_close=0.07 # if the recorded gripper is lower than 0.07, the robot is going to close during execution and if it larger, it is going to open
        
        self.gripper_width=0.08
        self.grip_command.goal.epsilon.inner = 0.3 #by having this big tollerance, the robot will adapt the grasp to any object dimension 
        self.grip_command.goal.epsilon.outer = 0.3 #by having this big tollerance, the robot will adapt the grasp to any object dimension 
        self.grip_command.goal.speed = 1
        self.grip_command.goal.force = 1
        self.grip_command.goal.width = 1

        self.attractor_distance_threshold = 0.08
        self.trajectory_distance_threshold = 0.08

        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_cartesian_pose",
                         PoseStamped, self.ee_pose_callback)
        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_cartesian_pose_global_frame",
                            PoseStamped, self.ee_pose_global_callback)
        rospy.Subscriber("panda_dual/" + str(self.name) + "_state_controller/joint_states", JointState,
                         self.joint_callback)
        rospy.Subscriber("/" + str(self.name) + "_franka_gripper/joint_states", JointState, self.gripper_callback)

        self.goal_pub = rospy.Publisher(
            "/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_equilibrium_pose", PoseStamped,
            queue_size=0)
        self.goal_pub_global = rospy.Publisher(
            "/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_equilibrium_pose_global_frame", PoseStamped,
            queue_size=0)
        self.configuration_pub = rospy.Publisher(
            "panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_nullspace", JointState,
            queue_size=0)

        self.gripper_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/grasp/goal", GraspActionGoal,
                                           queue_size=0)
        self.homing_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/homing/goal", HomingActionGoal,
                                          queue_size=0)
        self.stop_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/stop/goal", StopActionGoal,
                                          queue_size=0)

        self.nullspace_configuration_pub = rospy.Publisher("/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_nullspace", JointState,
                                          queue_size=0)
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == Key.esc:
            self.end = True

    def ee_pose_callback(self, data):
        self.cart_pose = data
    
    def ee_pose_global_callback(self, data):
        self.cart_pose_global = data
    # joint angle subscriber
    def joint_callback(self, data):
        self.joint_pos = data.position[0:7]

    # gripper state subscriber
    def gripper_callback(self, data):
        self.gripper_width = np.copy(data.position[0] + data.position[1])

    def move_gripper(self, width):
        if width < self.gripper_width_close and self.grip_command.goal.width != 0:
            self.grip_command.goal.width = 0
            self.gripper_pub.publish(self.grip_command)

        elif width > self.gripper_width_close and self.grip_command.goal.width != 1:
            self.grip_command.goal.width = 1
            self.gripper_pub.publish(self.grip_command)

    def home_gripper(self):
        self.homing_pub.publish(self.home_command)

    def set_stiffness(self, k_t1, k_t2, k_t3, k_r1, k_r2, k_r3):

        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_X": k_t1})
        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_Y": k_t2})
        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_Z": k_t3})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_X": k_r1})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_Y": k_r2})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_Z": k_r3})

    def Active(self):
        self.set_stiffness(1000.0, 1000.0, 1000.0, 30.0, 30.0, 30.0)

    def Passive(self):
        self.set_stiffness(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def set_attractor(self, pose_st, global_frame=False):
        # print("Pose: ", pose_st)
        if not global_frame:
            self.goal_pub.publish(pose_st)
        else:
            self.goal_pub_global.publish(pose_st)
    def set_configuration(self, joint):
        joint_des = Float32MultiArray()
        joint_des.data = np.array(joint).astype(np.float32)
        self.configuration_pub.publish(joint_des)

    def execute(self, global_frame=False, offset_z=0.0): 
        r = rospy.Rate(self.control_freq)
        self.Active()
        self.go_to_start(global_frame=global_frame)
        for i in range(self.recorded_traj_pose.shape[1]):
            if not global_frame:
                goal = self.recorded_traj_pose[0][i]
                goal.pose.position.z = goal.pose.position.z + offset_z
                self.goal_pub.publish(goal)

            else:
                goal = self.recorded_traj_pose_global[0][i] 
                goal.pose.position.z = goal.pose.position.z + offset_z
                self.goal_pub_global.publish(goal)
            

            
            r.sleep()
        
    def go_to_start(self, global_frame=False):
        if not global_frame:
            goal= self.recorded_traj_pose[0][0]
            self.go_to_3d(goal)
        else:
            goal= self.recorded_traj_pose_global[0][0]
            self.go_to_3d(goal, global_frame=True)

    def go_to_3d(self, data, global_frame=False):
        control_freq = 50
        r = rospy.Rate(control_freq)
        if not global_frame:
            start = [self.cart_pose.pose.position.x, self.cart_pose.pose.position.y, self.cart_pose.pose.position.z]
            start_ori = [self.cart_pose.pose.orientation.w, self.cart_pose.pose.orientation.x, self.cart_pose.pose.orientation.y,
                     self.cart_pose.pose.orientation.z]
        else:
            start = [self.cart_pose_global.pose.position.x, self.cart_pose_global.pose.position.y, self.cart_pose_global.pose.position.z]
            start_ori = [self.cart_pose_global.pose.orientation.w, self.cart_pose_global.pose.orientation.x, self.cart_pose_global.pose.orientation.y,
                     self.cart_pose_global.pose.orientation.z]
        q_start = np.quaternion(start_ori[0], start_ori[1], start_ori[2], start_ori[3])
        # interpolate from start to goal with attractor distance of approx 1 mm
        goal_ = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

        q_goal = np.quaternion(data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y,
                               data.pose.orientation.z)

        print("Moving to start position")
        print("global_frame: ", global_frame)
        print("Start position: ", start)
        print("Goal position: ", goal_)
        squared_dist = np.sum(np.subtract(start, goal_) ** 2, axis=0)
        dist = np.sqrt(squared_dist)
        interp_dist = 0.001  # [m]
        step_num = math.floor(dist / interp_dist)

        x = np.linspace(start[0], goal_[0], step_num)
        y = np.linspace(start[1], goal_[1], step_num)
        z = np.linspace(start[2], goal_[2], step_num)
        quat = np.slerp_vectorized(q_start, q_goal, 0)

        position = [x[0], y[0], z[0]]
        orientation = [quat.w, quat.x, quat.y, quat.z]

        pose_goal = self.pose_st_from_pos_ori(position, orientation)

        self.set_attractor(pose_goal, global_frame)
        print("Moving to start position")

        self.Active()
        for i in range(step_num):
            progress = (i + 1) / step_num
            bar_length = 40
            block = int(round(bar_length * progress))
            bar = "#" * block + "-" * (bar_length - block)
            print(f"\rProgress: [{bar}] {progress * 100:.2f}%", end="")
            position = [x[i], y[i], z[i]]
            quat = np.slerp_vectorized(q_start, q_goal, i / step_num)
            orientation = [quat.w, quat.x, quat.y, quat.z]
            pose_goal = self.pose_st_from_pos_ori(position, orientation)
            self.set_attractor(pose_goal, global_frame)
            r.sleep()

    def pose_st_from_pos_ori(self, pos, ori):
        pose_goal = PoseStamped()
        pose_goal.header.seq = 1
        pose_goal.header.stamp = rospy.Time.now()
        pose_goal.header.frame_id = "map"
        pose_goal.pose.position.x = pos[0]
        pose_goal.pose.position.y = pos[1]
        pose_goal.pose.position.z = pos[2]
        pose_goal.pose.orientation.w = ori[0]
        pose_goal.pose.orientation.x = ori[1]
        pose_goal.pose.orientation.y = ori[2]
        pose_goal.pose.orientation.z = ori[3]
        return pose_goal
    

    def traj_rec(self, active=False):
        time.sleep(1)
        r = rospy.Rate(self.control_freq)
        if not active:
            self.Passive()

        self.end = False

        print("Recording started. Press Esc to stop the recording")

        self.recorded_traj_pose = self.cart_pose
        self.recorded_traj_pose_global = self.cart_pose_global
        self.recorded_joint = self.joint_pos
        self.recorded_gripper = self.gripper_width
        self.recorded_stiffness_lin=[self.K_cart, self.K_cart, self.K_cart]
        self.recorded_stiffness_ori=[self.K_ori, self.K_ori, self.K_ori]

        while not self.end:
            
            self.recorded_traj_pose = np.c_[self.recorded_traj_pose, self.cart_pose]
            self.recorded_traj_pose_global = np.c_[self.recorded_traj_pose_global, self.cart_pose_global]
            self.recorded_joint = np.c_[self.recorded_joint, self.joint_pos]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.gripper_width]
            self.recorded_stiffness_lin=np.c_[self.recorded_stiffness_lin, [self.K_cart, self.K_cart, self.K_cart]]
            self.recorded_stiffness_ori=np.c_[self.recorded_stiffness_ori, [self.K_ori, self.K_ori, self.K_ori]]

            r.sleep()
            
    def save(self, name='last'):
        np.savez(str(pathlib.Path().resolve()) + '/data/' + str(name) + '.npz',
                 recorded_traj_pose=self.recorded_traj_pose,
                 recorded_traj_pose_global=self.recorded_traj_pose_global,   
                 recorded_gripper=self.recorded_gripper,
                 recorded_stiffness_lin=self.recorded_stiffness_lin,
                 recorded_stiffness_ori=self.recorded_stiffness_ori)

    def load(self, name='last'):
        data = np.load(str(pathlib.Path().resolve()) + '/data/' + str(name) + '.npz', allow_pickle=True)

        self.recorded_traj_pose = data['recorded_traj_pose']
        self.recorded_traj_pose_global = data['recorded_traj_pose_global']
        self.recorded_gripper = data['recorded_gripper']
        self.recorded_stiffness_lin = data['recorded_stiffness_lin']
        self.recorded_stiffness_ori = data['recorded_stiffness_ori']


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
        
        self.go_to_3d(goal)
        self.nullspace_configuration_pub.publish(ns_msg)
        self.set_K.update_configuration({f'{str(self.name)}_nullspace_stiffness':10})

        rospy.sleep(rospy.Duration(secs=5))

        self.set_K.update_configuration({f'{str(self.name)}_nullspace_stiffness':0})
