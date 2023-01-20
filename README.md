# Franka Bimaual Impedance Controller

For more info: https://frankaemika.github.io/docs/installation_linux.html
- Open a terminal pressing ctrl+alt+t

-In case you already have some versions of libfranka installed, remove them to avoid conflicts with:
```
sudo apt remove "*libfranka*"
sudo apt autoremove
```
Type the following commands to generate and build libfranka
```
cd
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

This last comand may take several minutes. 

Now create a workspace (here called catkin_ws) and install franka_ros in it
```
cd
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/<ros-distro>/setup.sh
catkin_init_workspace src
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
rosdep install --from-paths src --ignore-src --rosdistro <ros-distro> -y --skip-keys libfranka
```
- Finally, install the controllers inside the folder "franka_ros" and build the code:
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
roslaunch franka_bimanual_controllers dual_arm_cartesian_impedance_example_controller.launch robot_ips:="{panda_right/robot_ip: 172.16.0.2,panda_left/robot_ip: 172.16.0.3}" panda_right:=172.16.0.2 panda_left:=172.16.0.3
``` 
Warning: you must change the ip address with the ones of your setup!

You can now read the topic of each of the robot. For example: 
```
rostopic echo /panda_right_franka_gripper/joint_states
```
or
```
rostopic echo /panda_left_franka_gripper/joint_states
```
# How to run a simple learning from demonstration using python 
In a new terminal run 
```
python3 python/dual_server.py 
```
Then run the cells of python/main.py in an interactive way, one by one. 
Check this to learn how to do it with vscode: https://code.visualstudio.com/docs/python/jupyter-support-py 
