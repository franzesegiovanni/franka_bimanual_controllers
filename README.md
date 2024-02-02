# Franka Bimaual Impedance Controller
This repository was developed for the paper: 
Interactive Imitation Learning of Bimanual Movement Primitives, 

Giovanni Franzese, Leandro de Souza Rosa, Tim Verburg, Luka Peternel, Jens Kober. 

Published in IEEE/ASME Transactions on Mechatronics. 

If this code is useful in your research, please cite us:

```
Franzese, G., de Souza Rosa, L., Verburg, T., Peternel, L. and Kober, J., 2023. Interactive imitation learning of bimanual movement primitives. IEEE/ASME Transactions on Mechatronics.
```

- Install the controllers inside the folder "franka_ros" and build the code:
```
cd src/franka_ros
git clone https://github.com/franzesegiovanni/franka_bimanual_controllers.git
cd ../..
source /opt/ros/<ros-distro>/setup.bash
catkin_make -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
source devel/setup.sh
```

To start the controller use this command

```
roslaunch franka_bimanual_controllers dual_arm_cartesian_impedance_example_controller.launch robot_right_ip:=<ip_robot_right> robot_left_ip:=<ip_robot_left>

``` 

You can now read the topic of each of the robot. For example: 
```
rostopic echo /panda_right_franka_gripper/joint_states
```
or
```
rostopic echo /panda_left_franka_gripper/joint_states
```

For python codes, please refer to https://github.com/franzesegiovanni/SIMPLe/tree/main/SIMPLe_bimanual


For any problems or questions, feel free to open an issue in the repo or send an email to g.franzese@tudelft.nl
