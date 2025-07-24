// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include "sensor_msgs/JointState.h"
#include <franka_bimanual_controllers/dual_arm_compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Header.h>

namespace franka_bimanual_controllers {

/**
 * This container holds all data and parameters used to control one panda arm with a Cartesian
 * impedance control law tracking a desired target pose.
 */
struct FrankaDataContainer {
  std::unique_ptr<franka_hw::FrankaStateHandle>
      state_handle_;  ///< To read to complete robot state.
  std::unique_ptr<franka_hw::FrankaModelHandle>
      model_handle_;  ///< To have access to e.g. jacobians.
  std::vector<hardware_interface::JointHandle> joint_handles_;  ///< To command joint torques.

  Eigen::Matrix<double, 7, 1> nullspace_stiffness_{Eigen::Matrix<double, 7, 1>::Zero()}; ///< [Nm/rad] Joint-specific gains to track the desired
                                                                                         ///< nullspace joint configuration.

  const double delta_tau_max_{1.0};          ///< [Nm/ms] Maximum difference in joint-torque per
                                             ///< timestep. Used to saturated torque rates to ensure
                                             ///< feasible commands.
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;         ///< To track the target pose.

  Eigen::Matrix<double, 6, 6> cartesian_damping_;           ///< To damp cartesian motions.

  Eigen::Matrix<double, 7, 1> q_d_nullspace_;               ///< Target joint pose for nullspace

                                                            ///< motion. For now we track the
                                                            ///< initial joint pose.
  Eigen::Vector3d position_d_;               ///< Target position of the end effector.
  Eigen::Quaterniond orientation_d_;         ///< Target orientation of the end effector.

  Eigen::Vector3d position_other_arm_;               ///< Target position of the end effector.
  Eigen::Vector3d position_d_relative_;

  Eigen::Matrix<double, 6, 6> cartesian_stiffness_relative_;         ///< To track the target pose.
  Eigen::Matrix<double, 6, 6> cartesian_damping_relative_;
  Eigen::Matrix<double, 6, 1> force_torque;
};

/**
 * Controller class for ros_control that renders two decoupled Cartesian impedances for the
 * tracking of two target poses for the two endeffectors. The controller can be reparameterized at
 * runtime via dynamic reconfigure servers.
 */
class BiManualCartesianImpedanceControl
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  /**
   * Initializes the controller class to be ready to run.
   *
   * @param[in] robot_hw Pointer to a RobotHW class to get interfaces and resource handles.
   * @param[in] node_handle Nodehandle that allows getting parameterizations from the server and
   * starting subscribers.
   * @return True if the controller was initialized successfully, false otherwise.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;

  /**
   * Prepares the controller for the real-time execution. This method is executed once every time
   * the controller is started and runs in real-time.
   */
  void starting(const ros::Time&) override;

  /**
   * Computes the control-law and commands the resulting joint torques to the robot.
   *
   * @param[in] period The control period (here 0.001s).
   */
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Publisher for automatic error recovery
  ros::Publisher pub_error_recovery_;

  // Flags to track if arms are in an error state
  bool left_arm_in_error_{false};
  bool right_arm_in_error_{false};

  std::map<std::string, FrankaDataContainer>
      arms_data_;             ///< Holds all relevant data for both arms.
  std::string left_arm_id_;   ///< Name of the left arm, retrieved from the parameter server.
  std::string right_arm_id_;  ///< Name of the right arm, retrieved from the parameter server.

  ///< Transformation between base frames of the robots.
  Eigen::Affine3d Ol_T_Or_;  // NOLINT (readability-identifier-naming)
  ///< Target transformation between the two endeffectors.
  Eigen::Affine3d EEr_T_EEl_;  // NOLINT (readability-identifier-naming)
  ///< Transformation from the centering frame to the left end effector.
  Eigen::Affine3d EEl_T_C_{};

  std::atomic<bool> is_safe_{true}; ///< Safety flag (atomic for thread safety).
  ros::Time last_heartbeat_time_;     ///< Timestamp of the last received heartbeat.
  std::atomic<bool> initial_heartbeat_received_{false}; ///< Flag to indicate if the first heartbeat was received (atomic for thread safety).
  std::mutex heartbeat_mutex_; ///< Mutex to protect last_heartbeat_time_ access.

  ///< Publisher for the centering tracking frame of the coordinated motion.
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> center_frame_pub_;
  ///< Rate to trigger publishing the current pose of the centering frame.
  franka_hw::TriggerRate publish_rate_;
  Eigen::Matrix<float, 7, 1> stiff_;
  double delta_lim;
  /**
   * Saturates torque commands to ensure feasibility.
   *
   * @param[in] arm_data The data container of the arm.
   * @param[in] tau_d_calculated The raw command according to the control law.
   * @param[in] tau_J_d The current desired torque, read from the robot state.
   * @return The saturated torque command for the 7 joints of one arm.
   */
  Eigen::Matrix<double, 7, 1> saturateTorqueRateLeft(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> saturateTorqueRateRight(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)
  /**
   * Initializes a single Panda robot arm.
   *
   * @param[in] robot_hw A pointer the RobotHW class for getting interfaces and resource handles.
   * @param[in] arm_id The name of the panda arm.
   * @param[in] joint_names The names of all joints of the panda.
   * @return True if successful, false otherwise.
   */
  bool initArm(hardware_interface::RobotHW* robot_hw,
               const std::string& arm_id,
               const std::vector<std::string>& joint_names);

  /**
   * Computes the decoupled controller update for a single arm.
   *
   * @param[in] arm_data The data container of the arm to control.
   */
  void updateArmLeft();
  void updateArmRight();
  /**
   * Prepares all internal states to be ready to run the real-time control for one arm.
   *
   * @param[in] arm_data The data container of the arm to prepare for the control loop.
   */
  void startingArmLeft();
  void startingArmRight();
  ///< Dynamic reconfigure server
  std::unique_ptr<dynamic_reconfigure::Server<
      franka_combined_bimanual_controllers::dual_arm_compliance_paramConfig>>
      dynamic_server_compliance_param_;

  ///< Nodehandle for the dynamic reconfigure namespace
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;

  /**
   * Callback for updates on the parameterization of the controller in terms of stiffnesses.
   *
   * @param[in] config Data container for configuration updates.
   */
  void complianceParamCallback(
      franka_combined_bimanual_controllers::dual_arm_compliance_paramConfig& config,
      uint32_t /*level*/);

  ///< Target pose subscriber
  ros::Subscriber sub_equilibrium_pose_right_;
  void equilibriumPoseCallback_right(const geometry_msgs::PoseStampedConstPtr& msg);

  ros::Subscriber sub_equilibrium_pose_left_;
  void equilibriumPoseCallback_left(const geometry_msgs::PoseStampedConstPtr& msg);

  ros::Subscriber sub_equilibrium_distance_;
  void equilibriumPoseCallback_relative(const geometry_msgs::PoseStampedConstPtr& msg);

  ros::Subscriber sub_nullspace_right_;
  void equilibriumConfigurationCallback_right(const sensor_msgs::JointState::ConstPtr& joint);

  ros::Subscriber sub_nullspace_left_;
  void equilibriumConfigurationCallback_left(const  sensor_msgs::JointState::ConstPtr&  joint);

  ros::ServiceServer safety_service_server_;
  bool setSafetyCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  ros::Subscriber heartbeat_sub_;                                           // <<< Add heartbeat subscriber
  void heartbeatCallback(const std_msgs::Header::ConstPtr& msg);            // <<< Add heartbeat callback

  ros::Publisher pub_right;
  ros::Publisher pub_left;

  ros::Publisher pub_force_torque_right;
  ros::Publisher pub_force_torque_left;

  double joint_limits[7][2];
  double calculateTauJointLimit(double q_value, double threshold, double magnitude, double upper_bound, double lower_bound);


};

}  // namespace franka_bimanual_controllers
