// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_bimanual_controllers/bimanual_cartesian_impedance_controller.h>

#include <cmath>
#include <functional>
#include <memory>

#include <controller_interface/controller_base.h>
#include <eigen_conversions/eigen_msg.h>
#include <franka/robot_state.h>
#include <franka_bimanual_controllers/pseudo_inversion.h>
#include <franka_hw/trigger_rate.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "sensor_msgs/JointState.h"
namespace franka_bimanual_controllers {

bool BiManualCartesianImpedanceControl::initArm(
    hardware_interface::RobotHW* robot_hw,
    const std::string& arm_id,
    const std::vector<std::string>& joint_names) {
  FrankaDataContainer arm_data;
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "BiManualCartesianImpedanceControl: Error getting model interface from hardware");
    return false;
  }
  try {
    arm_data.model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "BiManualCartesianImpedanceControl: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "BiManualCartesianImpedanceControl: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data.state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "BiManualCartesianImpedanceControl: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "BiManualCartesianImpedanceControl: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data.joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "BiManualCartesianImpedanceControl: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }

  arm_data.position_d_.setZero();
  arm_data.orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  //arm_data.position_d_target_.setZero();
  //arm_data.orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  arm_data.cartesian_stiffness_.setZero();
  arm_data.cartesian_damping_.setZero();

  arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));

  return true;
}

bool BiManualCartesianImpedanceControl::init(hardware_interface::RobotHW* robot_hw,
                                                      ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;


  if (!node_handle.getParam("left/arm_id", left_arm_id_)) {
    ROS_ERROR_STREAM(
        "BiManualCartesianImpedanceControl: Could not read parameter left_arm_id_");
    return false;
  }
  std::vector<std::string> left_joint_names;
  if (!node_handle.getParam("left/joint_names", left_joint_names) || left_joint_names.size() != 7) {
    ROS_ERROR(
        "BiManualCartesianImpedanceControl: Invalid or no left_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("right/arm_id", right_arm_id_)) {
    ROS_ERROR_STREAM(
        "BiManualCartesianImpedanceControl: Could not read parameter right_arm_id_");
    return false;
  }

  std::vector<std::string> right_joint_names;
  if (!node_handle.getParam("right/joint_names", right_joint_names) ||
      right_joint_names.size() != 7) {
    ROS_ERROR(
        "BiManualCartesianImpedanceControl: Invalid or no right_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  bool left_success = initArm(robot_hw, left_arm_id_, left_joint_names);
  bool right_success = initArm(robot_hw, right_arm_id_, right_joint_names);

  sub_equilibrium_pose_right_ = node_handle.subscribe(
      "panda_right_equilibrium_pose", 20, &BiManualCartesianImpedanceControl::equilibriumPoseCallback_right, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_equilibrium_pose_left_ = node_handle.subscribe(
      "panda_left_equilibrium_pose", 20, &BiManualCartesianImpedanceControl::equilibriumPoseCallback_left, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_nullspace_right_ = node_handle.subscribe(
    "panda_right_nullspace", 20, &BiManualCartesianImpedanceControl::equilibriumConfigurationCallback_right, this,
    ros::TransportHints().reliable().tcpNoDelay());

  sub_nullspace_left_ = node_handle.subscribe(
    "panda_left_nullspace", 20, &BiManualCartesianImpedanceControl::equilibriumConfigurationCallback_left, this,
    ros::TransportHints().reliable().tcpNoDelay());

  sub_equilibrium_distance_ = node_handle.subscribe(
        "equilibrium_distance", 20, &BiManualCartesianImpedanceControl::equilibriumPoseCallback_relative, this,
        ros::TransportHints().reliable().tcpNoDelay());


  pub_right = node_handle.advertise<geometry_msgs::PoseStamped>("panda_right_cartesian_pose", 1);

  pub_left = node_handle.advertise<geometry_msgs::PoseStamped>("panda_left_cartesian_pose", 1);

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<
      franka_combined_bimanual_controllers::dual_arm_compliance_paramConfig>>(
      dynamic_reconfigure_compliance_param_node_);

  dynamic_server_compliance_param_->setCallback(boost::bind(
      &BiManualCartesianImpedanceControl::complianceParamCallback, this, _1, _2));


   return left_success && right_success;
}

void BiManualCartesianImpedanceControl::starting(const ros::Time& /*time*/) {
startingArmLeft();
startingArmRight();
}

void BiManualCartesianImpedanceControl::update(const ros::Time& /*time*/,
                                                        const ros::Duration& /*period*/) {
updateArmLeft();
updateArmRight();

}

void BiManualCartesianImpedanceControl::startingArmLeft() {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  auto& left_arm_data = arms_data_.at(left_arm_id_);

  franka::RobotState initial_state = left_arm_data.state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      left_arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set target point to current state
  left_arm_data.position_d_ = initial_transform.translation();
  left_arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  left_arm_data.position_d_ = initial_transform.translation();
  left_arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace target configuration to initial q
  left_arm_data.q_d_nullspace_ = q_initial;
}

void BiManualCartesianImpedanceControl::startingArmRight() {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  auto& right_arm_data = arms_data_.at(right_arm_id_);
  franka::RobotState initial_state = right_arm_data.state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      right_arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set target point to current state
  right_arm_data.position_d_ = initial_transform.translation();
  right_arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  right_arm_data.position_d_ = initial_transform.translation();
  right_arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace target configuration to initial q
  right_arm_data.q_d_nullspace_ = q_initial;
}

void BiManualCartesianImpedanceControl::updateArmLeft() {
  // get state variables
  auto& left_arm_data = arms_data_.at(left_arm_id_);
  auto& right_arm_data = arms_data_.at(right_arm_id_);
  franka::RobotState robot_state_left = left_arm_data.state_handle_->getRobotState();
  std::array<double, 49> inertia_array = left_arm_data.model_handle_->getMass();
  std::array<double, 7> coriolis_array = left_arm_data.model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      left_arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state_left.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state_left.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state_left.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state_left.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  franka::RobotState robot_state_right = right_arm_data.state_handle_->getRobotState();
  Eigen::Affine3d transform_right(Eigen::Matrix4d::Map(robot_state_right.O_T_EE.data()));
  Eigen::Vector3d position_right(transform_right.translation());
  // left_arm_data.position_other_arm_=position_right;
  // compute error to desired pose
  // position error
  geometry_msgs::PoseStamped msg_left;
  msg_left.pose.position.x=position[0];
  msg_left.pose.position.y=position[1];
  msg_left.pose.position.z=position[2];

  msg_left.pose.orientation.x=orientation.x();
  msg_left.pose.orientation.y=orientation.y();
  msg_left.pose.orientation.z=orientation.z();
  msg_left.pose.orientation.w=orientation.w();
  pub_left.publish(msg_left);

  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - left_arm_data.position_d_;

  Eigen::Matrix<double, 6, 1> error_relative;
  error_relative.head(3) << position - position_right; //todo change
  error_relative.tail(3).setZero();


  error_relative.head(3)<< error_relative.head(3) -left_arm_data.position_d_relative_;

  // orientation error
  if (left_arm_data.orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * left_arm_data.orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_joint_limit(7), null_space_error(7), tau_relative(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  franka_bimanual_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  null_space_error.setZero();
  null_space_error(0)=(left_arm_data.q_d_nullspace_(0) - q(0));
  null_space_error(1)=(left_arm_data.q_d_nullspace_(1) - q(1));
  null_space_error(2)=(left_arm_data.q_d_nullspace_(2) - q(2));
  null_space_error(3)=(left_arm_data.q_d_nullspace_(3) - q(3));
  null_space_error(4)=(left_arm_data.q_d_nullspace_(4) - q(4));
  null_space_error(5)=(left_arm_data.q_d_nullspace_(5) - q(5));
  null_space_error(6)=(left_arm_data.q_d_nullspace_(6) - q(6));
  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * (-left_arm_data.cartesian_stiffness_ * error -
                                      left_arm_data.cartesian_damping_ * (jacobian * dq)); 
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (left_arm_data.nullspace_stiffness_ * null_space_error -
                        (2.0 * sqrt(left_arm_data.nullspace_stiffness_)) * dq);

  //Avoid joint limits
  tau_joint_limit.setZero();
  if (q(0)>2.85)     { tau_joint_limit(0)=-5; }
  if (q(0)<-2.85)    { tau_joint_limit(0)=+5; }
  if (q(1)>1.7)      { tau_joint_limit(1)=-5; }
  if (q(1)<-1.7)     { tau_joint_limit(1)=+5; }
  if (q(2)>2.85)     { tau_joint_limit(2)=-5; }
  if (q(2)<-2.85)    { tau_joint_limit(2)=+5; }
  if (q(3)>-0.1)     { tau_joint_limit(3)=-5; }
  if (q(3)<-3.0)     { tau_joint_limit(3)=+5; }
  if (q(4)>2.85)     { tau_joint_limit(4)=-5; }
  if (q(4)<-2.85)    { tau_joint_limit(4)=+5; }
  if (q(5)>3.7)      { tau_joint_limit(5)=-5; }
  if (q(5)<-0.1)     { tau_joint_limit(5)=+5; }
  if (q(6)>2.8)      { tau_joint_limit(6)=-5; }
  if (q(6)<-2.8)     { tau_joint_limit(6)=+5; }

  tau_relative << jacobian.transpose() * (-left_arm_data.cartesian_stiffness_relative_ * error_relative-
                                      left_arm_data.cartesian_damping_relative_ * (jacobian * dq)); //TODO: MAKE THIS VELOCITY RELATIVE
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis + tau_joint_limit + tau_relative;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRateLeft(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    left_arm_data.joint_handles_[i].setCommand(tau_d(i));
  }
}

Eigen::Matrix<double, 7, 1> BiManualCartesianImpedanceControl::saturateTorqueRateLeft(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
    auto& left_arm_data = arms_data_.at(left_arm_id_);
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, left_arm_data.delta_tau_max_),
                                               -left_arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
}

void BiManualCartesianImpedanceControl::updateArmRight() {
  auto& left_arm_data = arms_data_.at(left_arm_id_);
  auto& right_arm_data = arms_data_.at(right_arm_id_);
  // get state variables
  franka::RobotState robot_state = right_arm_data.state_handle_->getRobotState();
  std::array<double, 49> inertia_array = right_arm_data.model_handle_->getMass();
  std::array<double, 7> coriolis_array = right_arm_data.model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      right_arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());


  franka::RobotState robot_state_left = left_arm_data.state_handle_->getRobotState();
  Eigen::Affine3d transform_left(Eigen::Matrix4d::Map(robot_state_left.O_T_EE.data()));
  Eigen::Vector3d position_left(transform_left.translation());
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - right_arm_data.position_d_;
  geometry_msgs::PoseStamped msg_right;
  msg_right.pose.position.x=position[0];
  msg_right.pose.position.y=position[1];
  msg_right.pose.position.z=position[2];

  msg_right.pose.orientation.x=orientation.x();
  msg_right.pose.orientation.y=orientation.y();
  msg_right.pose.orientation.z=orientation.z();
  msg_right.pose.orientation.w=orientation.w();
  pub_right.publish(msg_right);
  Eigen::Matrix<double, 6, 1> error_relative;
  error_relative.head(3) << position - position_left;
  error_relative.tail(3).setZero();
  error_relative.head(3)<< error_relative.head(3) -right_arm_data.position_d_relative_;

  // orientation error
  if (right_arm_data.orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * right_arm_data.orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_joint_limit(7), null_space_error(7), tau_relative(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  franka_bimanual_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  null_space_error.setZero();
  null_space_error(0)=(right_arm_data.q_d_nullspace_(0) - q(0));
  null_space_error(1)=(right_arm_data.q_d_nullspace_(1) - q(1));
  null_space_error(2)=(right_arm_data.q_d_nullspace_(2) - q(2));
  null_space_error(3)=(right_arm_data.q_d_nullspace_(3) - q(3));
  null_space_error(4)=(right_arm_data.q_d_nullspace_(4) - q(4));
  null_space_error(5)=(right_arm_data.q_d_nullspace_(5) - q(5));
  null_space_error(6)=(right_arm_data.q_d_nullspace_(6) - q(6));
  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * (-right_arm_data.cartesian_stiffness_ * error -
                                      right_arm_data.cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (right_arm_data.nullspace_stiffness_ * null_space_error -
                        (2.0 * sqrt(right_arm_data.nullspace_stiffness_)) * dq);

  //Avoid joint limits
  tau_joint_limit.setZero();
  if (q(0)>2.85)     { tau_joint_limit(0)=-5; }
  if (q(0)<-2.85)    { tau_joint_limit(0)=+5; }
  if (q(1)>1.7)      { tau_joint_limit(1)=-5; }
  if (q(1)<-1.7)     { tau_joint_limit(1)=+5; }
  if (q(2)>2.85)     { tau_joint_limit(2)=-5; }
  if (q(2)<-2.85)    { tau_joint_limit(2)=+5; }
  if (q(3)>-0.1)     { tau_joint_limit(3)=-5; }
  if (q(3)<-3.0)     { tau_joint_limit(3)=+5; }
  if (q(4)>2.85)     { tau_joint_limit(4)=-5; }
  if (q(4)<-2.85)    { tau_joint_limit(4)=+5; }
  if (q(5)>3.7)      { tau_joint_limit(5)=-5; }
  if (q(5)<-0.1)     { tau_joint_limit(5)=+5; }
  if (q(6)>2.8)      { tau_joint_limit(6)=-5; }
  if (q(6)<-2.8)     { tau_joint_limit(6)=+5; }

  tau_relative << jacobian.transpose() * (-right_arm_data.cartesian_stiffness_relative_ * error_relative-
                                      right_arm_data.cartesian_damping_relative_ * (jacobian * dq)); //TODO: MAKE THIS VELOCITY RELATIVE
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis+tau_joint_limit+tau_relative;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRateRight(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    right_arm_data.joint_handles_[i].setCommand(tau_d(i));
  }
}

Eigen::Matrix<double, 7, 1> BiManualCartesianImpedanceControl::saturateTorqueRateRight(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  auto& right_arm_data = arms_data_.at(right_arm_id_);
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, right_arm_data.delta_tau_max_),
                                               -right_arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
    }


void BiManualCartesianImpedanceControl::complianceParamCallback(
    franka_combined_bimanual_controllers::dual_arm_compliance_paramConfig& config,
    uint32_t /*level*/) {

   auto& left_arm_data = arms_data_.at(left_arm_id_);

   left_arm_data.cartesian_stiffness_.setIdentity();
   left_arm_data.cartesian_stiffness_(0,0)=config.panda_left_translational_stiffness_X;
   left_arm_data.cartesian_stiffness_(1,1)=config.panda_left_translational_stiffness_Y;
   left_arm_data.cartesian_stiffness_(2,2)=config.panda_left_translational_stiffness_Z;
   left_arm_data.cartesian_stiffness_(3,3)=config.panda_left_rotational_stiffness_X;
   left_arm_data.cartesian_stiffness_(4,4)=config.panda_left_rotational_stiffness_Y;
   left_arm_data.cartesian_stiffness_(5,5)=config.panda_left_rotational_stiffness_Z;

  left_arm_data.cartesian_damping_(0,0)=2.0 * sqrt(config.panda_left_translational_stiffness_X)*config.panda_left_damping_ratio;
  left_arm_data.cartesian_damping_(1,1)=2.0 * sqrt(config.panda_left_translational_stiffness_Y)*config.panda_left_damping_ratio;
  left_arm_data.cartesian_damping_(2,2)=2.0 * sqrt(config.panda_left_translational_stiffness_Z)*config.panda_left_damping_ratio;
  left_arm_data.cartesian_damping_(3,3)=2.0 * sqrt(config.panda_left_rotational_stiffness_X)*config.panda_left_damping_ratio;
  left_arm_data.cartesian_damping_(4,4)=2.0 * sqrt(config.panda_left_rotational_stiffness_Y)*config.panda_left_damping_ratio;
  left_arm_data.cartesian_damping_(5,5)=2.0 * sqrt(config.panda_left_rotational_stiffness_Z)*config.panda_left_damping_ratio;

  Eigen::AngleAxisd rollAngle_left(config.panda_left_stiffness_roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle_left(config.panda_left_stiffness_yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle_left(config.panda_left_stiffness_pitch, Eigen::Vector3d::UnitY());
  Eigen::Quaternion<double> q_left = rollAngle_left *  pitchAngle_left * yawAngle_left;
  Eigen::Matrix3d rotationMatrix_left = q_left.matrix();
  Eigen::Matrix3d rotationMatrix_transpose_left= rotationMatrix_left.transpose();
  left_arm_data.cartesian_stiffness_.topLeftCorner(3, 3) << rotationMatrix_left*left_arm_data.cartesian_stiffness_.topLeftCorner(3, 3)*rotationMatrix_transpose_left;
  left_arm_data.cartesian_stiffness_.bottomRightCorner(3, 3) << rotationMatrix_left*left_arm_data.cartesian_stiffness_.bottomRightCorner(3, 3)*rotationMatrix_transpose_left;


  left_arm_data.nullspace_stiffness_ = config.panda_left_nullspace_stiffness;

  left_arm_data.cartesian_stiffness_relative_.setIdentity();
  left_arm_data.cartesian_stiffness_relative_.topLeftCorner(3, 3)
      << config.coupling_translational_stiffness * Eigen::Matrix3d::Identity();
  left_arm_data.cartesian_stiffness_relative_.bottomRightCorner(3, 3)
      << 0.0 * Eigen::Matrix3d::Identity();

  left_arm_data.cartesian_damping_relative_.setIdentity();
  left_arm_data.cartesian_damping_relative_.topLeftCorner(3, 3)
      << 2* sqrt(config.coupling_translational_stiffness) * Eigen::Matrix3d::Identity();
  left_arm_data.cartesian_stiffness_relative_.bottomRightCorner(3, 3)
          << 0.0 * Eigen::Matrix3d::Identity();




  auto& right_arm_data = arms_data_.at(right_arm_id_);

  right_arm_data.cartesian_stiffness_.setIdentity();

  right_arm_data.cartesian_stiffness_(0,0)=config.panda_right_translational_stiffness_X;
  right_arm_data.cartesian_stiffness_(1,1)=config.panda_right_translational_stiffness_Y;
  right_arm_data.cartesian_stiffness_(2,2)=config.panda_right_translational_stiffness_Z;
  right_arm_data.cartesian_stiffness_(3,3)=config.panda_right_rotational_stiffness_X;
  right_arm_data.cartesian_stiffness_(4,4)=config.panda_right_rotational_stiffness_Y;
  right_arm_data.cartesian_stiffness_(5,5)=config.panda_right_rotational_stiffness_Z;

  right_arm_data.cartesian_damping_(0,0)=2.0 * sqrt(config.panda_right_translational_stiffness_X)*config.panda_right_damping_ratio;
  right_arm_data.cartesian_damping_(1,1)=2.0 * sqrt(config.panda_right_translational_stiffness_Y)*config.panda_right_damping_ratio;
  right_arm_data.cartesian_damping_(2,2)=2.0 * sqrt(config.panda_right_translational_stiffness_Z)*config.panda_right_damping_ratio;
  right_arm_data.cartesian_damping_(3,3)=2.0 * sqrt(config.panda_right_rotational_stiffness_X)*config.panda_right_damping_ratio;
  right_arm_data.cartesian_damping_(4,4)=2.0 * sqrt(config.panda_right_rotational_stiffness_Y)*config.panda_right_damping_ratio;
  right_arm_data.cartesian_damping_(5,5)=2.0 * sqrt(config.panda_right_rotational_stiffness_Z)*config.panda_right_damping_ratio;

  Eigen::AngleAxisd rollAngle_right(config.panda_right_stiffness_roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle_right(config.panda_right_stiffness_yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle_right(config.panda_right_stiffness_pitch, Eigen::Vector3d::UnitY());
  Eigen::Quaternion<double> q_right = rollAngle_right *  pitchAngle_right * yawAngle_right;
  Eigen::Matrix3d rotationMatrix_right = q_right.matrix();
  Eigen::Matrix3d rotationMatrix_transpose_right= rotationMatrix_right.transpose();
  right_arm_data.cartesian_stiffness_.topLeftCorner(3, 3) << rotationMatrix_right*right_arm_data.cartesian_stiffness_.topLeftCorner(3, 3)*rotationMatrix_transpose_right;
  right_arm_data.cartesian_stiffness_.bottomRightCorner(3, 3) << rotationMatrix_right*right_arm_data.cartesian_stiffness_.bottomRightCorner(3, 3)*rotationMatrix_transpose_right;

  right_arm_data.nullspace_stiffness_ = config.panda_right_nullspace_stiffness;


  right_arm_data.cartesian_stiffness_relative_.setIdentity();
  right_arm_data.cartesian_stiffness_relative_.topLeftCorner(3, 3)
      << config.coupling_translational_stiffness * Eigen::Matrix3d::Identity();
  right_arm_data.cartesian_stiffness_relative_.bottomRightCorner(3, 3)
      << 0.0 * Eigen::Matrix3d::Identity();

  right_arm_data.cartesian_damping_relative_.setIdentity();
  right_arm_data.cartesian_damping_relative_.topLeftCorner(3, 3)
      << 2* sqrt(config.coupling_translational_stiffness) * Eigen::Matrix3d::Identity();
  right_arm_data.cartesian_stiffness_relative_.bottomRightCorner(3, 3)
          << 0.0 * Eigen::Matrix3d::Identity();

}

void BiManualCartesianImpedanceControl::equilibriumPoseCallback_left(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  auto& left_arm_data = arms_data_.at(left_arm_id_);
  left_arm_data.position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_(left_arm_data.orientation_d_);
  left_arm_data.orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  // if (last_orientation_d_.coeffs().dot(left_arm_data.orientation_d_.coeffs()) < 0.0) {
  //   left_arm_data.orientation_d_.coeffs() << -left_arm_data.orientation_d_.coeffs();}
}

void BiManualCartesianImpedanceControl::equilibriumPoseCallback_right(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  auto& right_arm_data = arms_data_.at(right_arm_id_);
  right_arm_data.position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_(right_arm_data.orientation_d_);
  right_arm_data.orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  // if (last_orientation_d_.coeffs().dot(right_arm_data.orientation_d_.coeffs()) < 0.0) {
  //   right_arm_data.orientation_d_.coeffs() << -right_arm_data.orientation_d_.coeffs();}
}


void BiManualCartesianImpedanceControl::equilibriumPoseCallback_relative(
    const geometry_msgs::PoseStampedConstPtr& msg) {
      //This function is receiving the distance from the distance respect the right arm and the left
  auto& right_arm_data = arms_data_.at(right_arm_id_);
  auto&  left_arm_data = arms_data_.at(left_arm_id_);
  left_arm_data.position_d_relative_  << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  right_arm_data.position_d_relative_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  left_arm_data.position_d_relative_=-left_arm_data.position_d_relative_;
}

void BiManualCartesianImpedanceControl::equilibriumConfigurationCallback_right(const sensor_msgs::JointState::ConstPtr& joint) {

  auto& right_arm_data = arms_data_.at(right_arm_id_);
  std::vector<double> read_joint_right;
  read_joint_right= joint -> position;
  right_arm_data.q_d_nullspace_(0) = read_joint_right[0];
  right_arm_data.q_d_nullspace_(1) = read_joint_right[1];
  right_arm_data.q_d_nullspace_(2) = read_joint_right[2];
  right_arm_data.q_d_nullspace_(3) = read_joint_right[3];
  right_arm_data.q_d_nullspace_(4) = read_joint_right[4];
  right_arm_data.q_d_nullspace_(5) = read_joint_right[5];
  right_arm_data.q_d_nullspace_(6) = read_joint_right[6];
}

void BiManualCartesianImpedanceControl::equilibriumConfigurationCallback_left(const sensor_msgs::JointState::ConstPtr& joint) {

  auto& left_arm_data = arms_data_.at(left_arm_id_);
  std::vector<double> read_joint_left;
  read_joint_left= joint -> position;
  left_arm_data.q_d_nullspace_(0) = read_joint_left[0];
  left_arm_data.q_d_nullspace_(1) = read_joint_left[1];
  left_arm_data.q_d_nullspace_(2) = read_joint_left[2];
  left_arm_data.q_d_nullspace_(3) = read_joint_left[3];
  left_arm_data.q_d_nullspace_(4) = read_joint_left[4];
  left_arm_data.q_d_nullspace_(5) = read_joint_left[5];
  left_arm_data.q_d_nullspace_(6) = read_joint_left[6];

}
}  // namespace franka_bimanual_controllers

PLUGINLIB_EXPORT_CLASS(franka_bimanual_controllers::BiManualCartesianImpedanceControl,
                       controller_interface::ControllerBase)
