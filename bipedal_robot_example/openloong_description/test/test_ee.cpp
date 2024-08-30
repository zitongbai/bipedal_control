#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ros/ros.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include <urdf_parser/urdf_parser.h>

using namespace ocs2;

int main(int argc, char** argv){

    ros::init(argc, argv, "test_ee");
    ros::NodeHandle nh;
    std::string urdfFile, taskFile, referenceFile;
    nh.getParam("/urdfFile", urdfFile);
    nh.getParam("/taskFile", taskFile);
    nh.getParam("/referenceFile", referenceFile);

    std::vector<std::string> jointNames, contactNames3DoF;
    loadData::loadStdVector(taskFile, "model_settings.jointNames", jointNames, true);
    loadData::loadStdVector(taskFile, "model_settings.contactNames3DoF", contactNames3DoF, true);
    jointNames = std::vector<std::string>({
        "J_hip_l_roll", "J_hip_l_yaw", "J_hip_l_pitch", "J_knee_l_pitch", "J_ankle_l_pitch", "J_ankle_l_roll",
        "J_hip_r_roll", "J_hip_r_yaw", "J_hip_r_pitch", "J_knee_r_pitch", "J_ankle_r_pitch", "J_ankle_r_roll"
    });

    PinocchioInterface pinocchioInterface = centroidal_model::createPinocchioInterface(urdfFile, jointNames);
    CentroidalModelInfo centroidalModelInfo 
            = centroidal_model::createCentroidalModelInfo(
                    pinocchioInterface, 
                    centroidal_model::loadCentroidalType(taskFile),
                    centroidal_model::loadDefaultJointState(pinocchioInterface.getModel().nq-6, referenceFile), 
                    contactNames3DoF,
                    std::vector<std::string>());

    CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo);
    PinocchioEndEffectorKinematics endEffectorKinematics(pinocchioInterface, pinocchioMapping, contactNames3DoF);
    endEffectorKinematics.setPinocchioInterface(pinocchioInterface);

    const auto & model = pinocchioInterface.getModel();
    auto & data = pinocchioInterface.getData();
    vector_t q(model.nq);
    q << -0.00501472, 0.00108127, 1.07228, 0.0, 0.0, 0.0, 
        -0.11737, -0.0208852, 0.300253, -0.7342, 0.430147, 0.105293, 0.102646, 0.0207813, 0.258258, -0.639014, 0.373824, -0.088285;
    
    // vector_t q = vector_t::Zero(model.nq);
    // q(2) = 1.14;
    
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematics.getPosition(vector_t::Zero(model.nq+6));
    for (const auto & footPosition : feetPositions){
        std::cout << footPosition.transpose() << std::endl;
    }


    return 0;
}