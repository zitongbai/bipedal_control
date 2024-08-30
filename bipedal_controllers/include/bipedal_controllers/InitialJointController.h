/**
 * @file InitialJointController.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <unordered_map>

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>

#include <ocs2_bipedal_robot/common/ModelSettings.h>
#include <urdf/model.h>

#include <bipedal_common/hardware_interface/HybridJointInterface.h>
#include "bipedal_controllers/InitialJointControllerParamsConfig.h"

namespace ocs2{
namespace bipedal_robot{

class InitialJointPositionController : public controller_interface::Controller<HybridJointInterface> {
public:
    InitialJointPositionController() {};
    ~InitialJointPositionController() {};

    bool init(HybridJointInterface* hw, ros::NodeHandle &n);
    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);
    void starting(const ros::Time& /*time*/) {};


private:
    ModelSettings modelSettings_;

    ::urdf::ModelInterfaceSharedPtr urdfTree_;

    std::vector< std::string > jointNames_;
    unsigned int numJoints_;
    std::vector<scalar_t>  initialDesiredJointPos_;
    std::vector<scalar_t> upperLimit_;
    std::vector<scalar_t> lowerLimit_;
    std::vector<scalar_t> jointKp_;
    std::vector<scalar_t> jointKd_;

    // ros control interface
    std::vector<HybridJointHandle> jointHandles_;

    void enforceJointLimits(double &command, size_t index);

    // dynamic reconfiguration
    typedef dynamic_reconfigure::Server<bipedal_controllers::InitialJointControllerParamsConfig> DynamicReconfigServer;
    std::shared_ptr<DynamicReconfigServer> dynamicReconfigServer_;
    DynamicReconfigServer::CallbackType dynamicReconfigCallback_;
    void dynamicReconfigCallback(bipedal_controllers::InitialJointControllerParamsConfig& config, uint32_t level);


};

} // namespace bipedal_robot
} // namespace ocs2