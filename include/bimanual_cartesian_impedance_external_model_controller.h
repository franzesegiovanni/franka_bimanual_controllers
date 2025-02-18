// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka_bimanual_controllers/bimanual_cartesian_impedance_impedance_controller.h>

namespace franka_bimanual_controllers {

class BiManualCartesianImpedanceControlExternalModel : public BiManualCartesianImpedanceControl {
  private:
    std::string urdf_path_;
    pinocchio::Model model_pin_left_;
    pinocchio::Model model_pin_right_;    
    pinocchio::Data* data_pin_left_;
    pinocchio::Data* data_pin_right_;
    std::string frame_name_;
    int frame_id_;

  public:
    double* get_fk_left(franka::RobotState robot_state_left) override;
    double* get_fk_right(franka::RobotState robot_state_right) override; 
    std::array<double, 42> get_jacobian_left(franka::RobotState robot_state_left) override;
    std::array<double, 42> get_jacobian_right(franka::RobotState robot_state_right) override;
    void loadModel_left() override;
    void loadModel_right() override;
};

}  // namespace franka_bimanual_controllers
