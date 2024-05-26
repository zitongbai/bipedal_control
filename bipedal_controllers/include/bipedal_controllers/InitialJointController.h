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

    // ros control interface
    std::vector<HybridJointHandle> jointHandles_;

    std::vector<control_toolbox::Pid::Gains> pid_gains_;

    void enforceJointLimits(double &command, size_t index);
    
    
};

} // namespace bipedal_robot
} // namespace ocs2