# How to Implement your own robot

**The following instructions are out of data. It would be updated soon.**

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