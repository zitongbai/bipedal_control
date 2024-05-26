/**
 * @file SafetyChecker.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-25
 * @ref https://github.com/qiayuanl/legged_control
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_mpc/SystemObservation.h>

namespace ocs2 {
namespace bipedal_robot{

class SafetyChecker {
public: 
  explicit SafetyChecker(const CentroidalModelInfo& info) : info_(info) {}

  bool check(const SystemObservation& observation, const vector_t& /*optimized_state*/, const vector_t& /*optimized_input*/) {
    return checkOrientation(observation);
  }

protected: 
  const CentroidalModelInfo& info_;

  /**
   * @brief Check if the robot orientation is safe
   * 
   * @param observation 
   * @return true 
   * @return false 
   */
  bool checkOrientation(const SystemObservation& observation) {
    vector_t pose = centroidal_model::getBasePose(observation.state, info_);
    if (pose(5) > M_PI_2 || pose(5) < -M_PI_2) {
      std::cerr << "[SafetyChecker] Orientation safety check failed!" << std::endl;
      return false;
    }
    if (pose(4) > M_PI_2 || pose(4) < -M_PI_2) {
      std::cerr << "[SafetyChecker] Orientation safety check failed!" << std::endl;
      return false;
    }
    return true;
  }

};

}
}



