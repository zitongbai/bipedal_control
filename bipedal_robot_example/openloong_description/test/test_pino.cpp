#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ros/ros.h>

#include <urdf_parser/urdf_parser.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "test_ee");
    ros::NodeHandle nh;
    std::string urdfFile, taskFile, referenceFile;
    nh.getParam("/urdfFile", urdfFile);
    nh.getParam("/taskFile", taskFile);
    nh.getParam("/referenceFile", referenceFile);

    ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFile);
    if (urdfTree == nullptr) {
        throw std::invalid_argument("The file " + urdfFile + " does not contain a valid URDF model!");
    }

    // remove extraneous joints from urdf
    std::vector<std::string> jointNames = std::vector<std::string>({
        "J_hip_r_roll", "J_hip_r_yaw", "J_hip_r_pitch", "J_knee_r_pitch", "J_ankle_r_pitch", "J_ankle_r_roll",
        "J_hip_l_roll", "J_hip_l_yaw", "J_hip_l_pitch", "J_knee_l_pitch", "J_ankle_l_pitch", "J_ankle_l_roll"
    });

    std::vector<std::string> eeNames = std::vector<std::string>({
        "Link_ankle_l_roll", "Link_ankle_r_roll"
    });
    
    ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
    for (auto& jointPair : newModel->joints_) {
        if (std::find(jointNames.begin(), jointNames.end(), jointPair.first) == jointNames.end()) {
            jointPair.second->type = urdf::Joint::FIXED;
        }
    }

    // // add 6 DoF for the floating base
    // pinocchio::JointModelComposite jointComposite(2);
    // jointComposite.addJoint(pinocchio::JointModelTranslation());
    // jointComposite.addJoint(pinocchio::JointModelSphericalZYX());

    pinocchio::JointModelFreeFlyer rootJoint;
    
    pinocchio::ModelTpl<double> model;
    pinocchio::urdf::buildModel(newModel, rootJoint, model);

    std::cout << "model name: " << model.name << std::endl;
    std::cout << "model nq: " << model.nq << std::endl;

    pinocchio::Data data(model);

    Eigen::VectorXd q(model.nq);
    q << -0.00501472, 0.00108127, 1.07228, 0.0, 0.0, 0.0, 1.0, 
        -0.11737, -0.0208852, 0.300253, -0.7342, 0.430147, 0.105293, 0.102646, 0.0207813, 0.258258, -0.639014, 0.373824, -0.088285;
    
    forwardKinematics(model, data, q);
    updateFramePlacements(model, data);

    
    std::vector<Eigen::Vector3d> feetPositions;
    for(const auto & eeName : eeNames){
        pinocchio::FrameIndex frameId = model.getFrameId(eeName);
        feetPositions.emplace_back(data.oMf[frameId].translation());
    }

    for (const auto & footPosition : feetPositions){
        std::cout << footPosition.transpose() << std::endl;
    }

    // print all the joints in model
    for (const auto & joint : model.names){
        std::cout << joint << std::endl;
    }

    return 0;
}