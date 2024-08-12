/**
 * @file BipedalMujocoHWLoop.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bipedal_mujoco/hw/BipedalMujocoHWLoop.h"

namespace ocs2{
namespace bipedal_robot{

BipedalMujocoHWLoop::BipedalMujocoHWLoop(ros::NodeHandle& nh, std::shared_ptr<BipedalHW> hardware_interface, ::mujoco::Simulate & sim)
        : BipedalHWLoop(nh, hardware_interface), sim_(sim) {
  // nothing more to do
}

void BipedalMujocoHWLoop::update(){
  // check loop running condition 
  if(sim_.exitrequest.load()){
    loopRunning_ = false;
    ROS_INFO("[BipedalMujocoHWLoop] Exit request received, stopping loop");
    return;
  }

  // sleep for 1 ms or yield, to let main thread run
  //  yield results in busy wait - which has better timing but kills battery life
  if (sim_.run && sim_.busywait) {
    std::this_thread::yield();
  } else {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  BipedalHWLoop::update();

}

}
}