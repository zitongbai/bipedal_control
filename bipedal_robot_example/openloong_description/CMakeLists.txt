cmake_minimum_required(VERSION 3.0.2)
project(openloong_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ocs2_bipedal_robot
  ocs2_centroidal_model
  ocs2_core
  ocs2_ddp
  ocs2_ipm
  ocs2_mpc
  ocs2_msgs
  ocs2_oc
  ocs2_pinocchio_interface
  ocs2_robotic_assets
  ocs2_robotic_tools
  ocs2_ros_interfaces
  ocs2_sqp
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)


catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    pinocchio
)

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)


# add_executable(test_ee
#   test/test_pino.cpp
# )

# add_dependencies(test_ee ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(test_ee
#   ${catkin_LIBRARIES}
# )

