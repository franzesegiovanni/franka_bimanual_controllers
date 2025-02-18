#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <franka_bimanual_controllers/cartesian_variable_impedance_external_model_controller.h>
#include <ros/package.h>

namespace franka_bimanual_controllers {


void CartesianVariableImpedanceExternalModelController::loadModel_left() {
  std::string package_path = ros::package::getPath("franka_bimanual_controllers");
  urdf_path_ = package_path + "/urdf/panda_calibrated_left.urdf";
  ros::param::get("frame_name", frame_name_);

  std::cout << "Loading urdf into pinocchio as we are using the urdf model" << std::endl;
  pinocchio::urdf::buildModel(urdf_path_, model_pin_left_);
  frame_id_ = model_pin_.getFrameId(frame_name_);
  data_pin_left_ = new pinocchio::Data(model_pin_left_);
  std::cout << "Succesfully loaded the model and created the data pointer." << std::endl;
}

double* CartesianVariableImpedanceExternalModelController::get_fk_left(franka::RobotState robot_state_left)
{
  Eigen::Map<Eigen::Matrix<double, 9, 1>> q(robot_state_left.q.data());
  Eigen::VectorXd q_vector = Eigen::VectorXd::Map(q.data(), q.size());


  pinocchio::forwardKinematics(model_pin_left_, *data_pin_left_, q_vector);
  pinocchio::updateFramePlacement(model_pin_left_, *data_pin_left_, frame_id_);
  const auto& transformation = data_pin_left_->oMf[frame_id_];  // Get the transformation of the frame
  
  // Allocate memory for the result
  double* result = new double[16];
  std::memcpy(result, transformation.toHomogeneousMatrix().data(), 16 * sizeof(double));
  return result; // Caller is responsible for deleting the allocated memory
}

std::array<double, 42> CartesianVariableImpedanceExternalModelController::get_jacobian_left(franka::RobotState robot_state_left)
{
  Eigen::Map<Eigen::Matrix<double, 9, 1>> q(robot_state_left.q.data());
  Eigen::VectorXd q_vector = Eigen::VectorXd::Map(q.data(), q.size());
  Eigen::MatrixXd jacobian(6, model_pin_left_.nv);  // 6xnv matrix for spatial Jacobian
  jacobian.fill(0);  // Initialize to zero


  pinocchio::forwardKinematics_left(model_pin_left_, *data_pin_left_, q_vector);
  pinocchio::computeJointJacobians_left(model_pin_left_, *data_pin_left_, q_vector);
  pinocchio::getFrameJacobian_left(model_pin_left_, *data_pin_left_, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  std::array<double, 42> result;
  std::memcpy(result.data(), jacobian.data(), 42 * sizeof(double));
  return result;
}


void CartesianVariableImpedanceExternalModelController::loadModel_right() {
  std::string package_path = ros::package::getPath("franka_bimanual_controllers");
  urdf_path_ = package_path + "/urdf/panda_calibrated_right.urdf";
  ros::param::get("frame_name", frame_name_);

  std::cout << "Loading urdf into pinocchio as we are using the urdf model" << std::endl;
  pinocchio::urdf::buildModel(urdf_path_, model_pin_right_);
  frame_id_ = model_pin_right_.getFrameId(frame_name_);
  data_pin_right = new pinocchio::Data(model_pin_right_);
  std::cout << "Succesfully loaded the model and created the data pointer." << std::endl;
}

double* CartesianVariableImpedanceExternalModelController::get_fk_right(franka::RobotState robot_state_right)
{
  Eigen::Map<Eigen::Matrix<double, 9, 1>> q(robot_state_right.q.data());
  Eigen::VectorXd q_vector = Eigen::VectorXd::Map(q.data(), q.size());


  pinocchio::forwardKinematics(model_pin_right_, *data_pin_right, q_vector);
  pinocchio::updateFramePlacement(model_pin_right_, *data_pin_right, frame_id_);
  const auto& transformation = data_pin_right->oMf[frame_id_];  // Get the transformation of the frame
  
  // Allocate memory for the result
  double* result = new double[16];
  std::memcpy(result, transformation.toHomogeneousMatrix().data(), 16 * sizeof(double));
  return result; // Caller is responsible for deleting the allocated memory
}

std::array<double, 42> CartesianVariableImpedanceExternalModelController::get_jacobian_right(franka::RobotState robot_state_right)
{
  Eigen::Map<Eigen::Matrix<double, 9, 1>> q(robot_state_right.q.data());
  Eigen::VectorXd q_vector = Eigen::VectorXd::Map(q.data(), q.size());
  Eigen::MatrixXd jacobian(6, model_pin_right_.nv);  // 6xnv matrix for spatial Jacobian
  jacobian.fill(0);  // Initialize to zero


  pinocchio::forwardKinematics_right(model_pin_right_, *data_pin_right, q_vector);
  pinocchio::computeJointJacobians_right(model_pin_right_, *data_pin_right, q_vector);
  pinocchio::getFrameJacobian_right(model_pin_right_, *data_pin_right, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  std::array<double, 42> result;
  std::memcpy(result.data(), jacobian.data(), 42 * sizeof(double));
  return result;
}


}  // namespace franka_bimanual_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_bimanual_controllers::CartesianVariableImpedanceExternalModelController,
    controller_interface::ControllerBase
)
