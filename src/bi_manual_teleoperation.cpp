// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/dual_arm_cartesian_impedance_example_controller.h>

#include <cmath>
#include <functional>
#include <memory>

#include <controller_interface/controller_base.h>
#include <eigen_conversions/eigen_msg.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include <franka_hw/trigger_rate.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace franka_example_controllers {

bool DualArmCartesianImpedanceExampleController::initArm(
    hardware_interface::RobotHW* robot_hw,
    const std::string& arm_id,
    const std::vector<std::string>& joint_names) {
  FrankaDataContainer arm_data;
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    arm_data.model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceExampleController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data.state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceExampleController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceExampleController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data.joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "DualArmCartesianImpedanceExampleController: Exception getting joint handles: "
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

bool DualArmCartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                                      ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;


  if (!node_handle.getParam("left/arm_id", left_arm_id_)) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceExampleController: Could not read parameter left_arm_id_");
    return false;
  }
  std::vector<std::string> left_joint_names;
  if (!node_handle.getParam("left/joint_names", left_joint_names) || left_joint_names.size() != 7) {
    ROS_ERROR(
        "DualArmCartesianImpedanceExampleController: Invalid or no left_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("right/arm_id", right_arm_id_)) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceExampleController: Could not read parameter right_arm_id_");
    return false;
  }

  std::vector<std::string> right_joint_names;
  if (!node_handle.getParam("right/joint_names", right_joint_names) ||
      right_joint_names.size() != 7) {
    ROS_ERROR(
        "DualArmCartesianImpedanceExampleController: Invalid or no right_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  bool left_success = initArm(robot_hw, left_arm_id_, left_joint_names);
  bool right_success = initArm(robot_hw, right_arm_id_, right_joint_names);

  //boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)> callback =
      //boost::bind(&DualArmCartesianImpedanceExampleController::targetPoseCallback, this, _1);

  //ros::SubscribeOptions subscribe_options;
  //subscribe_options.init("centering_frame_target_pose", 1, callback);
  //subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  //sub_target_pose_left_ = node_handle.subscribe(subscribe_options);
  sub_equilibrium_pose_right_ = node_handle.subscribe(
      "/equilibrium_pose_right", 20, &DualArmCartesianImpedanceExampleController::equilibriumPoseCallback_right, this,      	      
      ros::TransportHints().reliable().tcpNoDelay());
  sub_equilibrium_pose_left_ = node_handle.subscribe(
      "/equilibrium_pose_left", 20, &DualArmCartesianImpedanceExampleController::equilibriumPoseCallback_left, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_stiffness_ = node_handle.subscribe(
    "/sub_stiffness_", 20, &DualArmCartesianImpedanceExampleController::equilibriumStiffnessCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  sub_nullspace_right_ = node_handle.subscribe(
    "/sub_nullspace_right_", 20, &DualArmCartesianImpedanceExampleController::equilibriumConfigurationCallback_right, this,
    ros::TransportHints().reliable().tcpNoDelay());

  sub_nullspace_left_ = node_handle.subscribe(
    "/sub_nullspace_left_", 20, &DualArmCartesianImpedanceExampleController::equilibriumConfigurationCallback_left, this,
    ros::TransportHints().reliable().tcpNoDelay());


   pub_stiff_update_ = node_handle.advertise<dynamic_reconfigure::Config>(
    "/dynamic_reconfigure_compliance_param_node/parameter_updates", 5);


  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<
      franka_combined_example_controllers::dual_arm_compliance_paramConfig>>(
      dynamic_reconfigure_compliance_param_node_);

  dynamic_server_compliance_param_->setCallback(boost::bind(
      &DualArmCartesianImpedanceExampleController::complianceParamCallback, this, _1, _2));

  // Get the transformation from right_O_frame to left_O_frame
  tf::StampedTransform transform;
  tf::TransformListener listener;
  try {
    if (listener.waitForTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0", ros::Time(0),
                                  ros::Duration(4.0))) {
      listener.lookupTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0", ros::Time(0),
                               transform);
    } else {
      ROS_ERROR(
          "DualArmCartesianImpedanceExampleController: Failed to read transform from %s to %s. "
          "Aborting init!",
          (right_arm_id_ + "_link0").c_str(), (left_arm_id_ + "_link0").c_str());
      return false;
    }
  } catch (tf::TransformException& ex) {
    ROS_ERROR("DualArmCartesianImpedanceExampleController: %s", ex.what());
    return false;
  }
  tf::transformTFToEigen(transform, Ol_T_Or_);  // NOLINT (readability-identifier-naming)

  // Setup publisher for the centering frame.
  publish_rate_ = franka_hw::TriggerRate(30.0);
  center_frame_pub_.init(node_handle, "centering_frame", 1, true);

  return left_success && right_success;
}

void DualArmCartesianImpedanceExampleController::startingArm(FrankaDataContainer& arm_data) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set target point to current state
  arm_data.position_d_ = initial_transform.translation();
  arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  arm_data.position_d_ = initial_transform.translation();
  arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace target configuration to initial q
  arm_data.q_d_nullspace_ = q_initial;
}

void DualArmCartesianImpedanceExampleController::starting(const ros::Time& /*time*/) {
  for (auto& arm_data : arms_data_) {
    startingArm(arm_data.second);
  }
  franka::RobotState robot_state_right =
      arms_data_.at(right_arm_id_).state_handle_->getRobotState();
  franka::RobotState robot_state_left = arms_data_.at(left_arm_id_).state_handle_->getRobotState();
  Eigen::Affine3d Ol_T_EEl(Eigen::Matrix4d::Map(  // NOLINT (readability-identifier-naming)
      robot_state_left.O_T_EE.data()));           // NOLINT (readability-identifier-naming)
  Eigen::Affine3d Or_T_EEr(Eigen::Matrix4d::Map(  // NOLINT (readability-identifier-naming)
      robot_state_right.O_T_EE.data()));          // NOLINT (readability-identifier-naming)
  EEr_T_EEl_ =
      Or_T_EEr.inverse() * Ol_T_Or_.inverse() * Ol_T_EEl;  // NOLINT (readability-identifier-naming)
  EEl_T_C_.setIdentity();
  Eigen::Vector3d EEr_r_EEr_EEl =  // NOLINT (readability-identifier-naming)
      EEr_T_EEl_.translation();    // NOLINT (readability-identifier-naming)
  EEl_T_C_.translation() = -0.5 * EEr_T_EEl_.inverse().rotation() * EEr_r_EEr_EEl;
}

void DualArmCartesianImpedanceExampleController::update(const ros::Time& /*time*/,
                                                        const ros::Duration& /*period*/) {
  for (auto& arm_data : arms_data_) {
    updateArm(arm_data.second);
  }
}

void DualArmCartesianImpedanceExampleController::updateArm(FrankaDataContainer& arm_data) {
  // get state variables
  franka::RobotState robot_state = arm_data.state_handle_->getRobotState();
  std::array<double, 49> inertia_array = arm_data.model_handle_->getMass();
  std::array<double, 7> coriolis_array = arm_data.model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

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

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - arm_data.position_d_;

  // orientation error
  if (arm_data.orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * arm_data.orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_joint_limit(7), null_space_error(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  franka_example_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  null_space_error.setZero();
  null_space_error(0)=(arm_data.q_d_nullspace_(0) - q(0));
  null_space_error(1)=(arm_data.q_d_nullspace_(1) - q(1));
  null_space_error(2)=(arm_data.q_d_nullspace_(2) - q(2));
  null_space_error(3)=(arm_data.q_d_nullspace_(3) - q(3));
  null_space_error(4)=(arm_data.q_d_nullspace_(4) - q(4));
  null_space_error(5)=(arm_data.q_d_nullspace_(5) - q(5));
  null_space_error(6)=(arm_data.q_d_nullspace_(6) - q(6));
  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * (-arm_data.cartesian_stiffness_ * error -
                                      arm_data.cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (arm_data.nullspace_stiffness_ * null_space_error -
                        (2.0 * sqrt(arm_data.nullspace_stiffness_)) * dq);
  
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
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis+tau_joint_limit;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(arm_data, tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    arm_data.joint_handles_[i].setCommand(tau_d(i));
  }
}

Eigen::Matrix<double, 7, 1> DualArmCartesianImpedanceExampleController::saturateTorqueRate(
    const FrankaDataContainer& arm_data,
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, arm_data.delta_tau_max_),
                                               -arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
}


void DualArmCartesianImpedanceExampleController::equilibriumStiffnessCallback(
    const std_msgs::Float32MultiArray::ConstPtr& stiffness_){

  int i = 0;
  // print all the remaining numbers
  for(std::vector<float>::const_iterator it = stiffness_->data.begin(); it != stiffness_->data.end(); ++it)
  {
    stiff_[i] = *it;
    i++;
  }
  auto& left_arm_data = arms_data_.at(left_arm_id_);
  left_arm_data.cartesian_stiffness_(0,0)=std::max(std::min(stiff_[0], float(1000.0)), float(0.0));
  left_arm_data.cartesian_stiffness_(1,1)=std::max(std::min(stiff_[0], float(1000.0)), float(0.0));
  left_arm_data.cartesian_stiffness_(2,2)=std::max(std::min(stiff_[0], float(1000.0)), float(0.0));

  left_arm_data.cartesian_damping_(0,0)=2.0 * sqrt(left_arm_data.cartesian_stiffness_(0,0));
  left_arm_data.cartesian_damping_(1,1)=2.0 * sqrt(left_arm_data.cartesian_stiffness_(1,1));
  left_arm_data.cartesian_damping_(2,2)=2.0 * sqrt(left_arm_data.cartesian_stiffness_(2,2));

  left_arm_data.cartesian_stiffness_(3,3)=std::max(std::min(stiff_[1], float(50.0)), float(0.0));
  left_arm_data.cartesian_stiffness_(4,4)=std::max(std::min(stiff_[1], float(50.0)), float(0.0));
  left_arm_data.cartesian_stiffness_(5,5)=std::max(std::min(stiff_[1], float(50.0)), float(0.0));

  left_arm_data.cartesian_damping_(3,3)=2.0 * sqrt(left_arm_data.cartesian_stiffness_(3,3));
  left_arm_data.cartesian_damping_(4,4)=2.0 * sqrt(left_arm_data.cartesian_stiffness_(4,4));
  left_arm_data.cartesian_damping_(5,5)=2.0 * sqrt(left_arm_data.cartesian_stiffness_(5,5)); 

  left_arm_data.nullspace_stiffness_= std::max(std::min(stiff_[2], float(20.0)), float(0.0));

  auto& right_arm_data = arms_data_.at(right_arm_id_);
  right_arm_data.cartesian_stiffness_(0,0)=std::max(std::min(stiff_[3], float(1000.0)), float(0.0));
  right_arm_data.cartesian_stiffness_(1,1)=std::max(std::min(stiff_[3], float(1000.0)), float(0.0));
  right_arm_data.cartesian_stiffness_(2,2)=std::max(std::min(stiff_[3], float(1000.0)), float(0.0));

  right_arm_data.cartesian_damping_(0,0)=2.0 * sqrt(right_arm_data.cartesian_stiffness_(0,0));
  right_arm_data.cartesian_damping_(1,1)=2.0 * sqrt(right_arm_data.cartesian_stiffness_(1,1));
  right_arm_data.cartesian_damping_(2,2)=2.0 * sqrt(right_arm_data.cartesian_stiffness_(2,2));

  right_arm_data.cartesian_stiffness_(3,3)=std::max(std::min(stiff_[4], float(50.0)), float(0.0));
  right_arm_data.cartesian_stiffness_(4,4)=std::max(std::min(stiff_[4], float(50.0)), float(0.0));
  right_arm_data.cartesian_stiffness_(5,5)=std::max(std::min(stiff_[4], float(50.0)), float(0.0));

  right_arm_data.cartesian_damping_(3,3)=2.0 * sqrt(right_arm_data.cartesian_stiffness_(3,3));
  right_arm_data.cartesian_damping_(4,4)=2.0 * sqrt(right_arm_data.cartesian_stiffness_(4,4));
  right_arm_data.cartesian_damping_(5,5)=2.0 * sqrt(right_arm_data.cartesian_stiffness_(5,5)); 

  right_arm_data.nullspace_stiffness_= std::max(std::min(stiff_[5], float(20.0)), float(0.0));

  dynamic_reconfigure::Config set_K_TL;
  dynamic_reconfigure::DoubleParameter param_TL_double;
  param_TL_double.name = "left_translational_stiffness";
  param_TL_double.value = stiff_[0];
  set_K_TL.doubles = {param_TL_double};
  pub_stiff_update_.publish(set_K_TL);

  dynamic_reconfigure::Config set_K_RL;
  dynamic_reconfigure::DoubleParameter param_RL_double;
  param_RL_double.name = "left_rotational_stiffness";
  param_RL_double.value = stiff_[1];
  set_K_RL.doubles = {param_RL_double};
  pub_stiff_update_.publish(set_K_TL);


  dynamic_reconfigure::Config set_K_NSL;
  dynamic_reconfigure::DoubleParameter param_NSL_double;
  param_NSL_double.name = "left_nullspace_stiffness";
  param_NSL_double.value = stiff_[2];
  set_K_NSL.doubles = {param_NSL_double};
  pub_stiff_update_.publish(set_K_NSL);

  dynamic_reconfigure::Config set_K_TR;
  dynamic_reconfigure::DoubleParameter param_TR_double;
  param_TR_double.name = "right_translational_stiffness";
  param_TR_double.value = stiff_[3];
  set_K_TR.doubles = {param_TR_double};
  pub_stiff_update_.publish(set_K_TR);

  dynamic_reconfigure::Config set_K_RR;
  dynamic_reconfigure::DoubleParameter param_RR_double;
  param_RR_double.name = "right_rotational_stiffness";
  param_RR_double.value = stiff_[4];
  set_K_RR.doubles = {param_RR_double};
  pub_stiff_update_.publish(set_K_RR);

  dynamic_reconfigure::Config set_K_NSR;
  dynamic_reconfigure::DoubleParameter param_NSR_double;
  param_NSR_double.name = "right_nullspace_stiffness";
  param_NSR_double.value = stiff_[5];
  set_K_NSR.doubles = {param_NSR_double};
  pub_stiff_update_.publish(set_K_NSR);

}



void DualArmCartesianImpedanceExampleController::complianceParamCallback(
    franka_combined_example_controllers::dual_arm_compliance_paramConfig& config,
    uint32_t /*level*/) {
  auto& left_arm_data = arms_data_.at(left_arm_id_);
  left_arm_data.cartesian_stiffness_.setIdentity();
  left_arm_data.cartesian_stiffness_.topLeftCorner(3, 3)
      << config.left_translational_stiffness * Eigen::Matrix3d::Identity();
  left_arm_data.cartesian_stiffness_.bottomRightCorner(3, 3)
      << config.left_rotational_stiffness * Eigen::Matrix3d::Identity();
  left_arm_data.cartesian_damping_.setIdentity();

  left_arm_data.cartesian_damping_.topLeftCorner(3, 3)
      << 2 * sqrt(config.left_translational_stiffness) * Eigen::Matrix3d::Identity();
  left_arm_data.cartesian_damping_.bottomRightCorner(3, 3)
      << 2 * sqrt(config.left_rotational_stiffness) * Eigen::Matrix3d::Identity();
  left_arm_data.nullspace_stiffness_ = config.left_nullspace_stiffness;

  auto& right_arm_data = arms_data_.at(right_arm_id_);
  right_arm_data.cartesian_stiffness_.setIdentity();
  right_arm_data.cartesian_stiffness_.topLeftCorner(3, 3)
      << config.right_translational_stiffness * Eigen::Matrix3d::Identity();
  right_arm_data.cartesian_stiffness_.bottomRightCorner(3, 3)
      << config.right_rotational_stiffness * Eigen::Matrix3d::Identity();
  right_arm_data.cartesian_damping_.setIdentity();

  right_arm_data.cartesian_damping_.topLeftCorner(3, 3)
      << 2 * sqrt(config.right_translational_stiffness) * Eigen::Matrix3d::Identity();
  right_arm_data.cartesian_damping_.bottomRightCorner(3, 3)
      << 2 * sqrt(config.right_rotational_stiffness) * Eigen::Matrix3d::Identity();
  right_arm_data.nullspace_stiffness_ = config.right_nullspace_stiffness;
}

void DualArmCartesianImpedanceExampleController::equilibriumPoseCallback_right(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  auto& right_arm_data = arms_data_.at(right_arm_id_);
  right_arm_data.position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_(right_arm_data.orientation_d_);
  right_arm_data.orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_.coeffs().dot(right_arm_data.orientation_d_.coeffs()) < 0.0) {
    right_arm_data.orientation_d_.coeffs() << -right_arm_data.orientation_d_.coeffs();  
}
}

void DualArmCartesianImpedanceExampleController::equilibriumPoseCallback_left(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  auto& left_arm_data = arms_data_.at(left_arm_id_);
  left_arm_data.position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_(left_arm_data.orientation_d_);
  left_arm_data.orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_.coeffs().dot(left_arm_data.orientation_d_.coeffs()) < 0.0) {
    left_arm_data.orientation_d_.coeffs() << -left_arm_data.orientation_d_.coeffs();  
}
}
void DualArmCartesianImpedanceExampleController::equilibriumConfigurationCallback_left( 
const std_msgs::Float32MultiArray::ConstPtr& joint) {
  auto& right_arm_data = arms_data_.at(left_arm_id_);
  int i = 0;
  for(std::vector<float>::const_iterator it = joint->data.begin(); it != joint->data.end(); ++it)
  {
    right_arm_data.q_d_nullspace_[i] = *it;
    i++;
  }
  return;    
}

void DualArmCartesianImpedanceExampleController::equilibriumConfigurationCallback_right( 
const std_msgs::Float32MultiArray::ConstPtr& joint) {
  auto& left_arm_data = arms_data_.at(right_arm_id_);
  int i = 0;
  for(std::vector<float>::const_iterator it = joint->data.begin(); it != joint->data.end(); ++it)
  {
    left_arm_data.q_d_nullspace_[i] = *it;
    i++;
  }
  return;    
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualArmCartesianImpedanceExampleController,
                       controller_interface::ControllerBase)
