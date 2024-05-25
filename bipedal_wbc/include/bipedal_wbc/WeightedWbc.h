/**
 * @file WeightedWbc.h
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-24
 * @ref https://github.com/qiayuanl/legged_control
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "bipedal_wbc/WbcBase.h"
namespace ocs2{
namespace bipedal_robot {
class WeightedWbc : public WbcBase {
public:
    explicit WeightedWbc(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
        : WbcBase(pinocchioInterface, info, eeKinematics) {};
    /**
     * @brief update the desired state, input, and measured state
     *  then solve the problem
     * 
     * @param stateDesired 
     * @param inputDesired 
     * @param rbdStateMeasured 
     * @param mode 
     * @param period 
     * @return vector_t 
     */
    vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;

    void loadTasksSetting(const std::string& taskFile, bool verbose) override;


protected:
    virtual Task formulateConstraints();
    virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);

private:
    scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;


};
}  // namespace bipedal
}  // namespace ocs2