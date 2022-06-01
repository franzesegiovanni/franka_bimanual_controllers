#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32
class LfD():
    def __init__(self):
        self.r=rospy.Rate(10)
        self.curr_pos=None
        self.width=None
        self.recorded_traj = None 
        self.recorded_gripper= None

        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)
        self.gripper_sub=rospy.Subscriber("/joint_states", JointState, self.gripper_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.grip_pub = rospy.Publisher('/gripper_online', Float32, queue_size=0)
        self.stiff_pub = rospy.Publisher('/stiffness', Float32MultiArray, queue_size=0) #TODO check the name of this topic 
    def ee_pos_callback(self, data):
        self.curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        #rospy.loginfo([data.x, data.y, data.z])
    def gripper_callback(self, data):
        self.width =data.position[7]+data.position[8]
        #rospy.loginfo(self.width)
    def traj_rec(self):
        # trigger for starting the recording
        trigger = 0.05
        stiff_des = Float32MultiArray()
        stiff_des.data = np.array([0.0, 0.0, 0.0, 30.0, 30.0, 30.0, 20.0]).astype(np.float32)
        self.stiff_pub.publish(stiff_des) 

        init_pos = self.curr_pos
        vel = 0
        while vel > trigger:
            vel = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
        
        self.recorded_traj = self.curr_pos
        #print("starting position:", self.curr_pos.shape)

        self.recorded_gripper= self.width
        # recorded_joint = joint_pos
        pygame.init()
        pygame.display.set_mode((440, 80))
        pygame.display.set_caption('Stop Trajectory Recording With Keypress')
        key_pressed = False
        while not key_pressed:
            now = time.time()            # get the time

            self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.width]
            self.r.sleep()

        pygame.display.quit()
        pygame.quit()

    # control robot to desired goal position
    def go_to_start(self):
        start = self.curr_pos
        goal_=np.array([self.recorded_traj[0][0], self.recorded_traj[1][0], self.recorded_traj[2][0]])
        print("goal:", goal_)
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
        
        print("x shape", x.shape)
        print("x", x)
        goal = PoseStamped()
        
        goal.pose.position.x = x[0]
        goal.pose.position.y = y[0]
        goal.pose.position.z = z[0]

        goal.pose.orientation.x = 1.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 0.0

        self.goal_pub.publish(goal)

        
        stiff_des = Float32MultiArray()

        stiff_des.data = np.array([600.0, 600.0, 600.0, 30.0, 30.0, 30,0, 20.0]).astype(np.float32)
        self.stiff_pub.publish(stiff_des)
        goal = PoseStamped()
        for i in range(step_num):
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = x[i]
            goal.pose.position.y = y[i]
            goal.pose.position.z = z[i]

            goal.pose.orientation.x = 1.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0
            self.goal_pub.publish(goal)
            self.r.sleep()   


    def execute(self):
        for i in range (self.recorded_traj.shape[1]):
            goal = PoseStamped()

            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = self.recorded_traj[0][i] 
            goal.pose.position.y = self.recorded_traj[1][i]
            goal.pose.position.z = self.recorded_traj[2][i]

            goal.pose.orientation.x = 1.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0

            self.goal_pub.publish(goal)
            
            grip_command = Float32()

            grip_command.data = self.recorded_gripper[0][i]

            self.grip_pub.publish(grip_command) 

            self.r.sleep()

#%%
    if __name__ == '__main__':
        rospy.init_node('LfD', anonymous=True)
#%%    
        LfD=LfD()
#%%
        input("Press Enter to start trajectory recording...")
        LfD.traj_rec() 

#%%
        LfD.go_to_start()

#%%
        LfD.execute()
#%%
