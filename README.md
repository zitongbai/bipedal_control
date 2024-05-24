# bipedal_control

bipedal_control is an NMPC framework for bipedal robot depending on [OCS2](https://github.com/leggedrobotics/ocs2)

# Installation

## Preparation

Before installation, make sure you have ROS noetic installed in Ubuntu 20.04. 

Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/) to use `catkin build` instead of `catkin_make`.

```bash
sudo apt-get install python3-catkin-tools
```

Create a catkin workspace: 
```bash
mkdir -p ~/bipedal_ws/src
cd ~/bipedal_ws/src
```

## OCS2

OCS2 is a huge monorepo; **DO NOT** try to compile the whole repo. You only need to compile `ocs2_legged_robot_ros` and
its dependencies following the step below.

1. You are supposed to clone the OCS2, pinocchio, and hpp-fcl as described in the documentation of OCS2.
   ```bash
   # Under the src directory of your catkin workspace
   # Clone OCS2
   git clone git@github.com:leggedrobotics/ocs2.git
   # Clone pinocchio
   git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
   # Clone hpp-fcl
   git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
   # Clone ocs2_robotic_assets
   git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
   # Install dependencies
   sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
   sudo apt install ros-noetic-grid-map-rviz-plugin
   ```
2. Compile the `ocs2_legged_robot_ros` package with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
   instead of `catkin_make`. It will take you several minutes.
   ```bash
   # Under catkin workspace
   cd ~/bipedal_ws
   catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
   catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization
   ```
3. Launch the example to test whether ocs2 is installed correctly.
   ```bash
    # Under catkin workspace
    # Source workspace
    source devel/setup.bash
    # Launch the example for DDP
    roslaunch ocs2_legged_robot_ros legged_robot_ddp.launch
    ```


   Ensure you can command the ANYmal as shown in
   the [document](https://leggedrobotics.github.io/ocs2/robotic_examples.html#legged-robot) and below.
   ![](https://leggedrobotics.github.io/ocs2/_images/legged_robot.gif)

## Build

Clone the repository to the `src` folder of your catkin workspace.

```bash
cd ~/bipedal_ws/src
git clone https://github.com/zitongbai/bipedal_control.git
```

Build the source code of `bipedal_control`.

```bash
cd ~/bipedal_ws
catkin build ocs2_bipedal_robot_ros
```

# Implemente your own robot

For implementation of your own robot, here are some steps to follow:

1. Create a folder under `bipedal_robot_example` to store some ROS packages specific to your robot. (e.g., `bipedal_robot_example/unitree_h1`)

2. Create a ROS package for the urdf model of your robot, or you can copy the out-of-the-box package containing the urdf model to here. (e.g., `bipedal_robot_example/unitree_h1/h1_description`). Make sure the urdf model is correct (you can use `check_urdf`) and can be visualized in RViz.

3. Create a ROS package for the configuration of ocs2 for your robot. (e.g., `bipedal_robot_example/unitree_h1/h1_ocs2_config`). It should contain the following contents: 
    * `gait.info`: the gait information of the robot, including several mode templates and the corresponding mode sequences.
    * `reference.info`: the reference information of the robot, it would be used in your command.
    * `task.info`
    
    You can copy these files from `bipedal_robot_example/unitree_h1/h1_ocs2_config` and modify them according to your robot. Details would be explained later. 

    You can also create the launch file in this package to start bipedal interface node and a dummy node to test the MPC trajectory planner. (you can refer to `bipedal_robot_example/unitree_h1/h1_ocs2_config/launch/bipedal_robot_sqp.launch`) 

4. If your robot does not have links in the sole, add virtual links to the urdf model of your robot. These links are used to construct contact constraints in MPC. You can refer to `right_sole_1_link`, etc. in `bipedal_robot_example/unitree_h1/h1_description/urdf/h1_with_sole.urdf` for an example.

5. Modify `jointNames` in `task.info` to match **ONLY** the **revolute legged** joint names of your robot. Upper body joints should not be included.

6. Modify `contactNames3DoF` in `task.info` to match the contact points (links defined in 4) of your robot.

7. Modify `initialState` in `task.info` and `defaultJointState` in `reference.info` to set the initial state of your robot.

8. Create a launch file like `/bipedal_robot_example/unitree_h1/h1_ocs2_config/launch/bipedal_robot_sqp.launch` to launch the bipedal interface node and a dummy node to test the MPC trajectory planner.

9. Modify `torqueLimitsTask` in `task.info`

# Acknowledgement

* [OCS2](https://github.com/leggedrobotics/ocs2): a C++ toolbox tailored for Optimal Control for Switched Systems.
* [legged_control](https://github.com/qiayuanl/legged_control.git):an NMPC-WBC legged robot control stack and framework.

