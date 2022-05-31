#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>

double width_right =0;
double width_left =0;
double width_old_left=1;
double width_old_right=1;
double flag_left =0;
double flag_right =0;
void Gripper_Left(const std_msgs::Float32::ConstPtr& msg)
{
  width_left=msg->data;
  flag_left = 1;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}
void Gripper_Right(const std_msgs::Float32::ConstPtr& msg)
{
  width_right=msg->data;
  flag_right = 1;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gripper");


  ros::NodeHandle gripper_left;
  ros::NodeHandle gripper_right;
  ros::Rate loop_rate(200);

  ros::Subscriber sub_left = gripper_left.subscribe("/gripper_left", 0, Gripper_Left);
  ros::Subscriber sub_right = gripper_right.subscribe("/gripper_right", 0, Gripper_Right);
  ros::Publisher pub_move_left = gripper_left.advertise<franka_gripper::MoveActionGoal>("/panda_left_franka_gripper/move/goal", 10);
  ros::Publisher pub_move_right = gripper_right.advertise<franka_gripper::MoveActionGoal>("/panda_right_franka_gripper/move/goal", 10);

  //ros::Publisher pub_stop = n.advertise<franka_gripper::StopAction>("/franka_gripper/stop", 1);
  ros::Publisher pub_grasp_left = gripper_left.advertise<franka_gripper::GraspActionGoal>("/panda_left_franka_gripper/grasp/goal", 10);
  ros::Publisher pub_grasp_right = gripper_right.advertise<franka_gripper::GraspActionGoal>("/panda_right_franka_gripper/grasp/goal", 10);
  franka_gripper::MoveActionGoal msg_move_left;
  franka_gripper::MoveActionGoal msg_move_right;
  franka_gripper::GraspActionGoal msg_grasp_left;
  franka_gripper::GraspActionGoal msg_grasp_right;
  msg_move_left.goal.speed = 10;
  msg_move_right.goal.speed = 10;
  msg_grasp_left.goal.speed = 10;
  msg_grasp_right.goal.speed = 10;

  //franka_gripper::StopActionGoal msg_stop;
  while (ros::ok())
  {
   if(flag_left==1)
   {
     if(width_left<=(width_old_left)) {
     msg_move_left.goal.width = width_left;
     pub_move_left.publish(msg_move_left);}
     else {
       msg_move_left.goal.width = width_left;
       pub_move_left.publish(msg_move_left);
       msg_grasp_left.goal.width = width_left;
       pub_grasp_left.publish(msg_grasp_left);}
     width_old_left=width_left;
     flag_left = 0;
    }
    if(flag_right==1)
   {
     if(width_right<=(width_old_right)) {
     msg_move_right.goal.width = width_right;
     pub_move_right.publish(msg_move_right);}
     else {
       msg_move_right.goal.width = width_right;
       pub_move_right.publish(msg_move_right);
       msg_grasp_right.goal.width = width_right;
       pub_grasp_right.publish(msg_grasp_right);}
     width_old_right=width_right;
     flag_right = 0;
    }



    ros::spinOnce();

    loop_rate.sleep();
  }





  //ros::spin();


  return 0;
}
