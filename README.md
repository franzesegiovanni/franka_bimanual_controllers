# franka_bimanual_controllers
to start the controller use this command

roslaunch franka_bimanual_controllers dual_arm_cartesian_impedance_example_controller.launch robot_ips:="{panda_right/robot_ip: 172.16.0.2,panda_left/robot_ip: 172.16.0.3}" panda_right:=172.16.0.2 panda_left:=172.16.0.3

rostopic echo /panda_right_franka_gripper/joint_states
rostopic echo /panda_left_franka_gripper/joint_states
