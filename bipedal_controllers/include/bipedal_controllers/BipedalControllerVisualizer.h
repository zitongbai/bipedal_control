/**
 * @file BipedalControllerVisualizer.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_bipedal_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>

namespace ocs2 {
namespace bipedal_robot {

/**
 * @brief A class to visualize the bipedal controller.
 *  including
 *      - 
 * 
 */
class BipedalControllerVisualizer {
public:
  /** Visualization settings (publicly available) */
  std::string frameId_ = "odom";              // Frame name all messages are published in
  scalar_t footMarkerDiameter_ = 0.03;        // Size of the spheres at the feet
  scalar_t footAlphaWhenLifted_ = 0.3;        // Alpha value when a foot is lifted.
  scalar_t forceScale_ = 1000.0;              // Vector scale in N/m
  scalar_t velScale_ = 5.0;                   // Vector scale in m/s
  scalar_t copMarkerDiameter_ = 0.03;         // Size of the sphere at the center of pressure
  scalar_t supportPolygonLineWidth_ = 0.005;  // LineThickness for the support polygon
  scalar_t trajectoryLineWidth_ = 0.01;       // LineThickness for trajectories
  std::vector<Color> feetColorMap_ = {Color::blue, Color::orange, Color::yellow, Color::purple};  // Colors for markers per feet

  /**
   * @brief Construct a new Bipedal Controller Visualizer object
   * 
   * @param pinocchioInterface 
   * @param centroidalModelInfo 
   * @param endEffectorKinematics 
   * @param nodeHandle 
   * @param maxUpdateFrequency 
   */
  BipedalControllerVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                            const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle,
                            scalar_t maxUpdateFrequency = 100.0);

  ~BipedalControllerVisualizer() = default;

  void update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command);
  
  /**
   * @brief setup ros node
   * 
   * @param nodeHandle 
   */
  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  void publishObservation(ros::Time timeStamp, const SystemObservation& observation);

  // void publishTrajectory(const std::vector<SystemObservation>& system_observation_array, scalar_t speed = 1.0);

  void publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories);

  void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                       const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule);

private:
  /**
   * @brief update tf from odom to base link
   * 
   * @param timeStamp 
   * @param basePose 
   */
  void publishBaseTransform(ros::Time timeStamp, const vector_t& basePose);
  
  /**
   * @brief publish markers of feet position, feet forces (planned), center of pressure, 
   *        support polygon
   * 
   * @param timeStamp 
   * @param contactFlags 
   * @param feetPositions 
   * @param feetForces 
   */
  void publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t& contactFlags, const std::vector<vector3_t>& feetPositions,
                               const std::vector<vector3_t>& feetForces) const;

  PinocchioInterface pinocchioInterface_;
  const CentroidalModelInfo centroidalModelInfo_;
  std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;

  tf::TransformBroadcaster tfBroadcaster_;

  ros::Publisher costDesiredBasePositionPublisher_;
  std::vector<ros::Publisher> costDesiredFeetPositionPublishers_;
  ros::Publisher stateOptimizedPublisher_;
  ros::Publisher currentStatePublisher_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;

  std::vector<std::string> jointNames_;
  std::string baseLinkName_;

  
};


} // namespace bipedal_control
} // namespace ocs2