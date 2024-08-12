/**
 * @file BipedalMujocoHWLoop.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "bipedal_hw/BipedalHWLoop.h"
#include "../src/mujoco/simulate.h"

namespace ocs2{
namespace bipedal_robot{

class BipedalMujocoHWLoop : public BipedalHWLoop{
  using Clock = std::chrono::high_resolution_clock;
  using Duration = std::chrono::duration<double>;

public:
  BipedalMujocoHWLoop(ros::NodeHandle& nh, std::shared_ptr<BipedalHW> hardware_interface, ::mujoco::Simulate & sim);

  // override the update function, because we have to use ::mujoco::Simulate here
  void update() override;

public:
  ::mujoco::Simulate & sim_;

};

} // namespace 
}


