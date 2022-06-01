#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import time

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32
import dynamic_reconfigure.client
from sys import exit
from pynput.keyboard import Listener, KeyCode

class LfD():
    def __init__(self):
        rospy.init_node('LfD', anonymous=True)
        self.r=rospy.Rate(10)
        self.step_change=0.15
        self.curr_pos=None
        self.width=None
        self.recorded_traj = None 
        self.recorded_gripper= None
        self.save_cartesian_position=None
        self.end = False
        self.recording_state=False
        self.recorded_experiment = None 
        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)
        self.gripper_sub=rospy.Subscriber("/joint_states", JointState, self.gripper_callback)  
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.grip_pub = rospy.Publisher('/gripper_online', Float32, queue_size=0)
        self.stiff_mat_pub_ = rospy.Publisher('/stiffness', Float32MultiArray, queue_size=0) #TODO check the name of this topic 
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True        
        if key == KeyCode.from_char('c'):
            self.save_cartesian_position = True    
    def ee_pos_callback(self, data):
        self.curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.curr_ori = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        if self.recording_state == True:
            self.recorded_experiment = np.c_[self.recorded_experiment, self.curr_pos]
            self.recorded_time = np.c_[self.recorded_time, time.time()]
    def gripper_callback(self, data):
        self.width =data.position[7]+data.position[8]

    def set_stiffness(self, k_t1, k_t2, k_t3,k_r1,k_r2,k_r3, k_ns):

        set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        set_K.update_configuration({"translational_stiffness_X": k_t1})
        set_K.update_configuration({"translational_stiffness_Y": k_t2})
        set_K.update_configuration({"translational_stiffness_Z": k_t3})        
        set_K.update_configuration({"rotational_stiffness_X": k_r1}) 
        set_K.update_configuration({"rotational_stiffness_Y": k_r2}) 
        set_K.update_configuration({"rotational_stiffness_Z": k_r3})
        set_K.update_configuration({"nullspace_stiffness": k_ns}) 

    def cart_rec_point(self):
        self.set_stiffness(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        self.recorded_traj = self.curr_pos
        self.recorded_ori = self.curr_ori
        self.recorded_gripper= self.width
        self.end=False
        while not(self.end):       
            if  self.save_cartesian_position==True: 
                self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
                self.recorded_ori = np.c_[self.recorded_ori, self.curr_ori]
                self.recorded_gripper = np.c_[self.recorded_gripper, self.width]
                print('Adding cartesian position') 
                time.sleep(0.1)
                self.save_cartesian_position=False     


        print('End of the demonstration')  

        np.savez('recorded_points', recorded_traj=self.recorded_traj , recorded_ori=self.recorded_ori, recorded_gripper=self.recorded_gripper)

    def cart_rec(self):
        trigger = 0.05

        init_pos = self.curr_pos
        init_ori = self.curr_ori
        vel = 0
        vel_ori=0
        self.set_stiffness(0.0,0.0,0.0, 0.0,0.0, 0.0, 0.0)
        while not(vel > trigger or vel_ori> trigger):
            vel     = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
            vel_ori = math.sqrt((self.curr_ori[0]-init_ori[0])**2 + (self.curr_ori[1]-init_ori[1])**2 + (self.curr_ori[2]-init_ori[2])**2)
        print('Start Recording demonstration')
        self.recorded_traj = self.curr_pos
        self.recorded_ori = self.curr_ori
        self.recorded_gripper= self.width
        self.end=False
        while not(self.end):
            now = time.time()      

            self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
            self.recorded_ori = np.c_[self.recorded_ori, self.curr_ori]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.width]

            self.r.sleep()

        print('End of the demonstration') 

        np.savez('recorded_points', recorded_traj=self.recorded_traj , recorded_ori=self.recorded_ori, recorded_gripper=self.recorded_gripper)

    # control robot to desired goal position
    def go_to_start_cart(self):
        start = self.curr_pos
        start_ori = self.curr_ori
        goal_=np.array([self.recorded_traj[0][0], self.recorded_traj[1][0], self.recorded_traj[2][0]])#TODO Check this
        print("goal:", goal_)
        goal_ori_=np.array([self.recorded_ori[0][0], self.recorded_ori[1][0], self.recorded_ori[2][0], self.recorded_ori[3][0]])
        print("goal_ori:", goal_ori_)
        # interpolate from start to goal with attractor distance of approx 1 cm
        squared_dist = np.sum(np.subtract(start, goal_)**2, axis=0)
        dist = np.sqrt(squared_dist)
        print("dist", dist)
        interp_dist = 0.01  # [m]
        step_num = math.floor(dist / interp_dist)
        print("num of steps", step_num)
        x = np.linspace(start[0], goal_[0], step_num)
        y = np.linspace(start[1], goal_[1], step_num)
        z = np.linspace(start[2], goal_[2], step_num)
        rot_x= np.linspace(start_ori[0], goal_ori_[0], step_num)
        rot_y= np.linspace(start_ori[1], goal_ori_[1], step_num)
        rot_z= np.linspace(start_ori[2], goal_ori_[2], step_num)
        rot_w= np.linspace(start_ori[3], goal_ori_[3], step_num)

        goal = PoseStamped()
        goal.pose.position.x = x[0]
        goal.pose.position.y = y[0]
        goal.pose.position.z = z[0]

        goal.pose.orientation.x = rot_x[0]
        goal.pose.orientation.y = rot_y[0]
        goal.pose.orientation.z = rot_z[0]
        goal.pose.orientation.w = rot_w[0]

        self.goal_pub.publish(goal)

        self.set_stiffness(600.0, 600.0, 600.0, 30.0, 30.0, 30.0, 0.0)
        goal = PoseStamped()
        for i in range(step_num):
            now = time.time()            # get the time
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = x[i]
            goal.pose.position.y = y[i]
            goal.pose.position.z = z[i]

            goal.pose.orientation.x = rot_x[i]
            goal.pose.orientation.y = rot_y[i]
            goal.pose.orientation.z = rot_z[i]
            goal.pose.orientation.w = rot_w[i]
            self.goal_pub.publish(goal)
            self.r.sleep()   

        #self.set_stiffness(200.0, 200.0, 200.0, 30.0, 30.0, 30.0, 10.0)


    def track_position_cartesian(self):
        self.set_stiffness(2000.0, 2000.0, 2000.0, 0.0, 0.0, 0.0, 0.0)

    def load_file(self):
        goal_points = np.load('/home/pandarobot/Desktop/Advance_controllers_workspace/src/franka_ros/franka_advanced_controllers/python/recorded_points.npz')
        self.recorded_traj = goal_points['recorded_traj']
        self.recorded_ori = goal_points['recorded_ori']
        self.recorded_gripper = goal_points['recorded_gripper']

    def execute_step_change(self):
        self.set_stiffness(400.0, 400.0, 400.0, 0.0, 0.0, 0.0, 0.0) 
        self.recorded_experiment= self.curr_pos
        self.recording_state = True
        self.recorded_time=time.time()
        goal = PoseStamped()

        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = self.recorded_traj[0,0]
        goal.pose.position.y = self.recorded_traj[1,0]
        goal.pose.position.z = self.recorded_traj[2,0]

        goal.pose.orientation.x = self.recorded_ori[0,0]
        goal.pose.orientation.y = self.recorded_ori[1,0]
        goal.pose.orientation.z = self.recorded_ori[2,0]
        goal.pose.orientation.w = self.recorded_ori[3,0]

        self.goal_pub.publish(goal)
        print('goal position before step change: ', goal.pose.position)
        goal.pose.position.x = self.recorded_traj[0,0]+ self.step_change
        goal.pose.position.y = self.recorded_traj[1,0]+ self.step_change
        goal.pose.position.z = self.recorded_traj[2,0]+ self.step_change

        goal.pose.orientation.x = self.recorded_ori[0,0]
        goal.pose.orientation.y = self.recorded_ori[1,0]
        goal.pose.orientation.z = self.recorded_ori[2,0]
        goal.pose.orientation.w = self.recorded_ori[3,0]
        print('goal position after step change: ', goal.pose.position)
        time.sleep(1.0)
        self.goal_pub.publish(goal)
        time.sleep(3.0)
        #for i in range(np.shape(self.recorded_joint)[1]): 

    def experiment_step(self):
        set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        damp=[0.3, 0.5, 0.7071, 1.0, 1.5]
        for i in range(len(damp)):
            set_K.update_configuration({"damping_ratio_translation": 1.5 }) 
            self.go_to_start_cart()
            #self.track_position_cartesian()
            time.sleep(5)
            set_K.update_configuration({"damping_ratio_translation": damp[i] })
            self.execute_step_change()    
            self.recording_state = False
            np.savez('recorded_experiments_'+str(damp[i]), recorded_position=self.recorded_experiment, time_stamp=self.recorded_time) 
    


    def execute_cart_points(self):
        self.set_stiffness(200.0, 200.0, 200.0, 30.0, 30.0, 30.0, 10.0)
        for i in range (self.recorded_traj.shape[1]):
            goal = PoseStamped()

            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = self.recorded_traj[0,i]
            goal.pose.position.y = self.recorded_traj[1,i]
            goal.pose.position.z = self.recorded_traj[2,i]

            goal.pose.orientation.x = self.recorded_ori[0,i]
            goal.pose.orientation.y = self.recorded_ori[1,i]
            goal.pose.orientation.z = self.recorded_ori[2,i]
            goal.pose.orientation.w = self.recorded_ori[3,i]

            self.goal_pub.publish(goal)
            
            grip_command = Float32()
            grip_command.data = self.recorded_gripper[0,i]
            self.grip_pub.publish(grip_command) 

            time.sleep(5) 

    def execute_cart(self):
        self.set_stiffness(400.0, 400.0, 400.0, 30.0, 30.0, 30.0, 0.0)
        for i in range (self.recorded_traj.shape[1]):
            goal = PoseStamped()

            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = self.recorded_traj[0][i] 
            goal.pose.position.y = self.recorded_traj[1][i]
            goal.pose.position.z = self.recorded_traj[2][i]

            goal.pose.orientation.x = self.recorded_ori[0][i]
            goal.pose.orientation.y = self.recorded_ori[1][i]
            goal.pose.orientation.z = self.recorded_ori[2][i]
            goal.pose.orientation.w = self.recorded_ori[3][i]

            self.goal_pub.publish(goal)
            
            grip_command = Float32()

            grip_command.data = self.recorded_gripper[0,i]

            self.grip_pub.publish(grip_command) 

            self.r.sleep()                  
    
#%%    
LfD=LfD()
#%%
LfD.cart_rec_point()  
#%%
LfD.load_file()
#%%
LfD.go_to_start_cart()

#%%
LfD.execute_cart_points()

#%%
LfD.cart_rec() 

#%%
LfD.track_position_cartesian()
#%%
LfD.execute_step_change()
#%%
LfD.go_to_start_cart()

#%%
LfD.execute_cart()

# %%
LfD.experiment_step()


# %%
