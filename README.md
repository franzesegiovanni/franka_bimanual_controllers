# Franka Bimaual Impedance Controller
This repository was developed for the paper: 
Interactive Imitation Learning of Bimanual Movement Primitives.

Published in IEEE/ASME Transactions on Mechatronics. 

### What is this repository useful for? 
This code is build on top of [Franka Bimanual Cartesian Impedance Control](https://github.com/frankaemika/franka_ros/blob/develop/franka_example_controllers/src/dual_arm_cartesian_impedance_example_controller.cpp). 
This code has some other desirable features for controlling the two arms independently and set desired relative stiffness to constraint the movement of the two arms, for example when picking up an object.

Features:

- **Joint limit repulsion for each arm**: The controller has a joint limit repulsion feature that allows the robot to avoid joint limits. This is useful when the robot is controlled by a human operator.

- **Anisotropic stiffness for each arm**: The controller allows the user to set different stiffness values for each axis, both linear and angular.

- **Read the robot position and orientation of each arm**: The controller reads the robot position and orientation and publish them in the topic `/panda_dual/bimanual_cartesian_impedance_control/panda_left_cartesian_pose` or `/panda_dual/bimanual_cartesian_impedance_control/panda_right_cartesian_pose`  The message type is geometry_msgs/PoseStamped.
- **Change the orientation of the stiffness matrix**: For each arm, the stiffness matrix can be rotated. This is possible directly in `dynamic_reconfigure`. This is useful when for example you want to have an high stiffness in a certain diection that is not aligned with the world frame. One application can be bimanual [dressing](https://www.youtube.com/watch?v=0_cJqZmZIS4).

- **Publish the desired attractor for each arm**: the topic /equilibrium_pose is the topic where you can publish the deried pose of the robot. The message type is geometry_msgs/PoseStamped.

- **Set the desired attractor relative distance**: The controller allows the user to set the desired attractor relative distance. This is going to mechanically couple the two arm with a spring. The message type is geometry_msgs/PoseStamped. The current implementation only uses a linear coupling spring between the two arms.
This will work is the robot are set in the same orietation. The topic value should be `cartesian_postion_right - cartesian_postion_left`. You can set the stiffness of the coupling spring in the rqt_reconfigure.

- **Safety feature**: The attractor distance are clipped inside the controller. You can set the clipping value in the rqt_reconfigure. This enusre that the robot will not move too fast when a too far attractor is published.

- **Read external forces**: The controller reads the external forces and torques from the robot and publish them in the topic `/force_torque_left_ext` and `/force_torque_right_ext`. The message type is geometry_msgs/WrenchStamped. This value is already filtered as it is also compensating for the gravity, Coriolis and friction forces. 

- **Launch the grippers and read the gripper states** 
The current code is set to launch the grippers and read the gripper states. The gripper states are published in the topic `/panda_right_franka_gripper/joint_states` and `/panda_left_franka_gripper/joint_states`. The message type is sensor_msgs/JointState.

### Installation 
- Install Libfranka and franka_ros from [here](https://frankaemika.github.io/docs/installation_linux.html)

- Go the the catkin_ws where you install franka_ros

Be sure to not install the develop branch of fraka_ros that can contain some bugs. 
For example install the the noetic branch that will be the next stable release.
```
cd catkin_ws/src
git clone -b noetic-devel https://github.com/frankaemika/franka_ros.git
```

- Install the franka bimanual controller:

```
git clone https://github.com/franzesegiovanni/franka_bimanual_impedance_controller.git
cd .. 
catkin build -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

To start the controller use this command

```
roslaunch franka_bimanual_controllers dual_arm_cartesian_impedance_example_controller.launch robot_right_ip:=<ip_robot_right> robot_left_ip:=<ip_robot_left> arm_id:=<arm_id>
```

where <ip_robot_right> and <ip_robot_left> are the ip of the right and left robot respectively. The arm_id is the id of the robot that you have in the bimanual setup, i.e. panda or fr3. This code does not support hybrid panda-fr3 setup. 
The arm_id is important to change the joint repulsion behaviour.

### Learning from demonstration

For python codes, please refer to https://github.com/franzesegiovanni/SIMPLe/tree/main/SIMPLe_bimanual


# Cite us!

If this code is useful in your research, please cite us:

```
Franzese, G., de Souza Rosa, L., Verburg, T., Peternel, L. and Kober, J., 2023. Interactive Imitation Learning of Bimanual Movement Primitives. IEEE/ASME Transactions on Mechatronics.
```
