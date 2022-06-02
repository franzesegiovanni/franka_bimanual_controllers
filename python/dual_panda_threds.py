"""
Authors: Giovanni Franzese & Anna Mészáros, May 2022
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#!/usr/bin/env python
import rospy
import math
import numpy as np
import time
import pandas as pd
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Point, WrenchStamped, PoseStamped, Vector3
import dynamic_reconfigure.client
from std_msgs.msg import Float32MultiArray, Bool, Float32

from pynput.keyboard import Listener, KeyCode

class DualPanda():
    def __init__(self, arm_id_right='panda_right',arm_id_left='panda_left'):
        self.control_freq=100
        self.rec_freq=10
        rospy.init_node('DualArmControl', anonymous=True)
        rospy.Subscriber("/spacenav/offset", Vector3, self.teleop_callback)
        rospy.Subscriber("/spacenav/joy", Joy, self.btns_callback)
        self.Panda_right=Panda(arm_id_right)
        self.Panda_left=Panda(arm_id_left)
    # Start keyboard listener
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True   
       # spacemouse joystick subscriber

    def teleop_callback(self, data):
        self.feedback = [data.x, data.y, data.z]
    # spacemouse buttons subscriber
    def btns_callback(self, data):
        self.left_btn = data.buttons[0]
        self.right_btn = data.buttons[1]
    
    def Kinesthetic_Demonstration_BiManual(self, trigger=0.005): 
        r=rospy.Rate(self.rec_freq)
        self.Panda_left.set_stiffness(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.Panda_right.set_stiffness(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        self.end = False
        init_pos_right = self.Panda_right.cart_pos
        init_pos_left = self.Panda_left.cart_pos
        vel_right = 0
        vel_left = 0
        print("Move robot to start recording.")

        while vel_right < trigger and vel_left < trigger :
            vel_right = math.sqrt((self.Panda_right.cart_pos[0]-init_pos_right[0])**2 + (self.Panda_right.cart_pos[1]-init_pos_right[1])**2 + (self.Panda_right.cart_pos[2]-init_pos_right[2])**2)
            vel_left =  math.sqrt((self.Panda_left.cart_pos[0]-init_pos_left[0])**2 + (self.Panda_left.cart_pos[1]-init_pos_left[1])**2 + (self.Panda_left.cart_pos[2]-init_pos_left[2])**2)
        print("Recording started. Press e to stop.")

        self.recorded_traj_dual = [self.Panda_right.cart_pos, self.Panda_left.cart_pos] 
        self.recorded_joint_dual= [self.Panda_right.joint_pos, self.Panda_left.joint_pos] 
        while not self.end:

            self.recorded_traj_dual = np.c_[self.recorded_traj, [self.Panda_right.cart_pos, self.Panda_left.cart_pos] ]
            self.recorded_joint_dual = np.c_[self.recorded_joint, [self.Panda_right.joint_pos, self.Panda_left.joint_pos]]
            r.sleep()


class Panda():

    def __init__(self, arm_id=''):
        self.rec_freq=10
        self.control_freq=100
        self.name=arm_id
        self.K_ori  = 30.0
        self.K_cart = 600.0
        self.K_null = 0.0
        self.start = True
        self.end = False
        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/"+str(self.name)+"_cartesian_pose", PoseStamped, self.ee_pose_callback)
        rospy.Subscriber("panda_dual/"+str(self.name)+"_state_controller/joint_states", JointState, self.joint_callback)
        rospy.Subscriber("panda_dual/"+str(self.name)+"_state_controller/joint_states", JointState, self.gripper_callback)

        self.goal_pub  = rospy.Publisher("/panda_dual/bimanual_cartesian_impedance_controller/"+str(self.name)+"_equilibrium_pose", PoseStamped, queue_size=0)
        self.configuration_pub = rospy.Publisher("panda_dual/bimanual_cartesian_impedance_controller/"+str(self.name)+ "_nullspace",JointState, queue_size=0)
        self.configuration_pub = rospy.Publisher(str(self.name)+ "_gripper",Float32, queue_size=0)

        rospy.Subscriber("panda_dual/"+str(self.name)+"/goto", PoseStamped, self.go_to_3d)
        rospy.Subscriber("panda_dual/"+str(self.name)+"/execute", Bool, self.execute)

        self.goto_pub = rospy.Publisher("panda_dual/"+str(self.name)+"/goto", PoseStamped, queue_size=0)
        self.execute_pub = rospy.Publisher("panda_dual/"+str(self.name)+"/execute", Bool, queue_size=0)

        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True

    def ee_pose_callback(self, data):
        self.cart_pos = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.cart_ori = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

    # joint angle subscriber
    def joint_callback(self, data):
        self.joint_pos = data.position[0:7]


    # gripper state subscriber
    def gripper_callback(self, data):
        self.gripper_pos = data.position[7:9]

    def set_stiffness(self, k_t1, k_t2, k_t3,k_r1,k_r2,k_r3, k_ns):
        
        set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        set_K.update_configuration({str(self.name)+"_translational_stiffness_X": k_t1})
        set_K.update_configuration({str(self.name)+"_translational_stiffness_Y": k_t2})
        set_K.update_configuration({str(self.name)+"_translational_stiffness_Z": k_t3})        
        set_K.update_configuration({str(self.name)+"_rotational_stiffness_X": k_r1}) 
        set_K.update_configuration({str(self.name)+"_rotational_stiffness_Y": k_r2}) 
        set_K.update_configuration({str(self.name)+"_rotational_stiffness_Z": k_r3})
        set_K.update_configuration({str(self.name)+"_nullspace_stiffness": k_ns})    

    def set_attractor(self,pos,quat):
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = pos[0]
        goal.pose.position.y = pos[1]
        goal.pose.position.z = pos[2]

        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        self.goal_pub.publish(goal)

    def set_configuration(self,joint):
        joint_des=Float32MultiArray()
        joint_des.data= np.array(joint).astype(np.float32)
        self.configuration_pub.publish(joint_des)

    def execute_traj(self):
        goal = Bool()
        goal.data=True
        self.execute_pub.publish(goal)

    def execute(self, start):
        r=rospy.Rate(self.rec_freq)

        if start.data is True:
            self.set_stiffness(400.0, 400.0, 400.0, 30.0, 30.0, 30.0, 0.0)

            for i in range (self.recorded_traj.shape[1]):
                position=[self.recorded_traj[0][i],self.recorded_traj[1][i],self.recorded_traj[2][i]]
                orientation=[self.recorded_ori[0][i], self.recorded_ori[1][i], self.recorded_ori[2][i], self.recorded_ori[3][i]]
                self.set_attractor(position,orientation)
            
                # grip_command = Float32()
                # grip_command.data = self.recorded_gripper[0,i]
                # self.grip_pub.publish(grip_command) 

                r.sleep()
        start.data=False

    def go_to_start(self):
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.recorded_traj[0][0]
        goal.pose.position.y = self.recorded_traj[1][0]
        goal.pose.position.z = self.recorded_traj[2][0]

        goal.pose.orientation.x = self.recorded_ori[0][0]
        goal.pose.orientation.y = self.recorded_ori[1][0]
        goal.pose.orientation.z = self.recorded_ori[2][0]
        goal.pose.orientation.w = self.recorded_ori[3][0]

        self.goto_pub.publish(goal)

    def go_to_3d(self, data):
        start = self.cart_pos
        start_ori=self.cart_ori
        r=rospy.Rate(self.control_freq)
        # interpolate from start to goal with attractor distance of approx 1 mm
        goal_ = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        goal_ori_ = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        squared_dist = np.sum(np.subtract(start, goal_)**2, axis=0)
        dist = np.sqrt(squared_dist)
        interp_dist = 0.001  # [m]
        step_num = math.floor(dist / interp_dist)

        x = np.linspace(start[0], goal_[0], step_num)
        y = np.linspace(start[1], goal_[1], step_num)
        z = np.linspace(start[2], goal_[2], step_num)
        rot_x= np.linspace(start_ori[0], goal_ori_[0], step_num)
        rot_y= np.linspace(start_ori[1], goal_ori_[1], step_num)
        rot_z= np.linspace(start_ori[2], goal_ori_[2], step_num)
        rot_w= np.linspace(start_ori[3], goal_ori_[3], step_num)
        position=[x[0],y[0],z[0]]
        orientation=[rot_x[0], rot_y[0], rot_z[0], rot_w[0]]
        self.set_attractor(position, orientation)

        # pos_stiff=[self.K_cart, self.K_cart, self.K_cart]
        # rot_stiff=[self.K_ori, self.K_ori, self.K_ori]
        # null_stiff=[0]
        self.set_stiffness(self.K_cart, self.K_cart, self.K_cart, self.K_ori, self.K_ori, self.K_ori, 0)

        # send attractors to controller
        for i in range(step_num):
            position=[x[i],y[i],z[i]]
            orientation=[rot_x[i], rot_y[i], rot_z[i], rot_w[i]]
            self.set_attractor(position,orientation)
            r.sleep()

    def Kinesthetic_Demonstration(self, trigger=0.005): 
        r=rospy.Rate(self.rec_freq)
        self.set_stiffness(0,0,0,0,0,0,0)

        self.end = False
        init_pos = self.cart_pos
        vel = 0
        print("Move robot to start recording.")
        while vel < trigger:
            vel = math.sqrt((self.cart_pos[0]-init_pos[0])**2 + (self.cart_pos[1]-init_pos[1])**2 + (self.cart_pos[2]-init_pos[2])**2)

        print("Recording started. Press e to stop.")

        self.recorded_traj = self.cart_pos
        self.recorded_ori= self.cart_ori
        self.recorded_joint= self.joint_pos
        while not self.end:

            self.recorded_traj = np.c_[self.recorded_traj, self.cart_pos]
            self.recorded_ori= np.c_[self.recorded_ori, self.cart_ori]
            self.recorded_joint = np.c_[self.recorded_joint, self.joint_pos]
            r.sleep()
        
