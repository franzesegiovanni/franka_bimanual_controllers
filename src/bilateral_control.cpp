#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <franka/exception.h>
#include <franka_bimanual_controllers/bimanual_cartesian_impedance_controller.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include "franka_msgs/FrankaState.h"


std::array<double, 3> Position_left;
std::array<double, 4> Orientation_left;
std::array<double, 3> Position_right;
std::array<double, 4> Orientation_right;
std::array<double, 7> Nullspace_right;
std::array<double, 7> Nullspace_left;

void Read_Position_left_Callback(franka_msgs::FrankaState robot_state) //Probably here is missing a pointer but, if I add the pointer the code is not compiling 
{
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Vector3d position(transform.translation()); 
  Eigen::Quaterniond orientation(transform.linear());
  Position_left[0]= position[0];
  Position_left[1]= position[1];
  Position_left[2]= position[2];
  Nullspace_left[0]= q(0);
  Nullspace_left[1]= q(1);
  Nullspace_left[2]= q(2);
  Nullspace_left[3]= q(3);
  Nullspace_left[4]= q(4);
  Nullspace_left[5]= q(5);
  Nullspace_left[6]= q(6);
  Orientation_left[0] = orientation.x();  //x
  Orientation_left[1] = orientation.y();  //y
  Orientation_left[2] = orientation.z();  //z
  Orientation_left[3] = orientation.w();  //w
}

void Read_Position_right_Callback(franka_msgs::FrankaState robot_state) //Probably here is missing a pointer but, if I add the pointer the code is not compiling 
{
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Vector3d position(transform.translation()); 
  Eigen::Quaterniond orientation(transform.linear());
  Position_right[0]= position[0];
  Position_right[1]= position[1];
  Position_right[2]= position[2];
  Nullspace_right[0]= q(0);
  Nullspace_right[1]= q(1);
  Nullspace_right[2]= q(2);
  Nullspace_right[3]= q(3);
  Nullspace_right[4]= q(4);
  Nullspace_right[5]= q(5);
  Nullspace_right[6]= q(6);
  Orientation_right[0] = orientation.x();  //x
  Orientation_right[1] = orientation.y();  //y
  Orientation_right[2] = orientation.z();  //z
  Orientation_right[3] = orientation.w();  //w
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Position_EE");


  ros::NodeHandle node_position;
  ros::Rate loop_rate(1000);

  ros::Subscriber sub_right =     node_position.subscribe("/panda_dual/panda_right_state_controller/franka_states", 1, Read_Position_right_Callback);

  ros::Subscriber sub_left =      node_position.subscribe("/panda_dual/panda_left_state_controller/franka_states", 1, Read_Position_left_Callback);


  ros::Publisher  pub_right =    node_position.advertise<geometry_msgs::PoseStamped>("/equilibrium_pose_right", 1);


  ros::Publisher  pub_right_ns = node_position.advertise<sensor_msgs::JointState>("/nullspace_right_", 1);


  ros::Publisher  pub_left =     node_position.advertise<geometry_msgs::PoseStamped>("/equilibrium_pose_left", 1); 


  ros::Publisher  pub_left_ns =  node_position.advertise<sensor_msgs::JointState>("/nullspace_left_", 1); 


   geometry_msgs::PoseStamped msg_right;
   geometry_msgs::PoseStamped msg_left;
   sensor_msgs::JointState msg_left_ns;	
   sensor_msgs::JointState msg_right_ns;
  

  while (ros::ok())
  {
   

    //Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    
    msg_right.pose.position.x=Position_right[0];
    msg_right.pose.position.y=Position_right[1];
    msg_right.pose.position.z=Position_right[2];

    msg_right.pose.orientation.x=Orientation_right[0];
    msg_right.pose.orientation.y=Orientation_right[1];
    msg_right.pose.orientation.z=Orientation_right[2];

    msg_left_ns.position={Nullspace_left[0],Nullspace_left[1],Nullspace_left[2],Nullspace_left[3],Nullspace_left[4],Nullspace_left[5],Nullspace_left[6]} ;

    msg_left.pose.position.x=Position_left[0];
    msg_left.pose.position.y=Position_left[1];
    msg_left.pose.position.z=Position_left[2];

    msg_left.pose.orientation.x=Orientation_left[0];
    msg_left.pose.orientation.y=Orientation_left[1];
    msg_left.pose.orientation.z=Orientation_left[2];
    msg_left.pose.orientation.w=Orientation_left[3];

    msg_right_ns.position={Nullspace_right[0],Nullspace_right[1],Nullspace_right[2],Nullspace_right[3],Nullspace_right[4],Nullspace_right[5],Nullspace_right[6]} ;
  

    //This next lines are useful for creating the bilateral control 
    pub_left.publish(msg_right);
    pub_right.publish(msg_left);
    pub_left_ns.publish(msg_right_ns);
    pub_right_ns.publish(msg_left_ns);	
    ros::spinOnce();

    loop_rate.sleep();
  }
  

  //ros::spin();


  return 0;
}
