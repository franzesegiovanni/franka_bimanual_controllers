// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka_bimanual_controllers/bimanual_cartesian_impedance_impedance_controller.h>

namespace franka_bimanual_controllers {

class BiManualCartesianImpedanceControlExternalModel : public BiManualCartesianImpedanceControl {
  public:
    void loadModel() override;
    double* get_fk(franka::RobotState robot_state) override; 
    std::array<double, 42> get_jacobian(franka::RobotState robot_state) override;
};

}  // namespace franka_bimanual_controllers
