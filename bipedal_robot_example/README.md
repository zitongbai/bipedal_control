# How to Implement your own robot

## 1 Preparation

First you should prepare the urdf file and coresponding mesh files of your robot. If you want to simulate in Mujoco, the xml file of your robot should also be prepared.

## 2 Create a new package for your robot

Create a new package for your robot under the `bipedal_robot_example` folder. For example, if your robot is named `xxx`, you can create a package named `xxx_description`

```bash
# under the bipedal_robot_example folder
catkin_create_pkg xxx_description roscpp rospy
```

create the following folers under the `xxx_description` folder

* `urdf`: put the urdf file of your robot here
* `meshes`: put the mesh files of your robot here
* `mjcf`: put the mujoco xml file of your robot here
* `launch`: launch files
* `config`: configuration files
* `scripts`: scripts

make sure the paths inside the urdf and mjcf files are correct (you can use relative path). For example, the path of the mesh files should be like `package://xxx_description/meshes/base_link.stl`.

You can use rviz to visualize the urdf file of your robot, by running a launch file similar to that in `bipedal_robot_example/unitree_h1/h1_description/launch/display.launch`.
It is very useful when you want to check joint rotation directions, link positions, etc.

## 3 Create modified urdf files

### Create a new urdf file with sole links

Create a new copy of the urdf file of your robot (you can name it like `xxx_with_sole.urdf`), and modify it according to the following instructions.

We need to add links in feet for calculating the end effector kinematics and constructing contact constraints. So you should add 2 virtual links in each sole of your robot:

* left_sole_1_link
* left_sole_2_link
* right_sole_1_link
* right_sole_2_link

(you can name it in your own style)

Use `check_urdf` to view the urdf tree of the robot and add the sole links in the end of the leg chains. For example: 

```xml
<joint name="left_sole_1_joint" type="fixed">
    <origin xyz="0.19 0.0 -0.06" rpy="0 0 0"/>
    <parent link="left_ankle_link"/>
    <child link="left_sole_1_link"/>
</joint>
<link name="left_sole_1_link">
    <inertial>
        <mass value="0.01" />
        <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001" />
    </inertial>
    <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
        <sphere radius="0.001"/>
        </geometry>
    </visual>
    <collision>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
        <sphere radius="0.001"/>
        </geometry>
    </collision>
</link>
```

Note that the inertial properties of the sole links are necessary. You can assign a small mass and inertia to them.

You can:
* use `check_urdf` to check if the urdf file is correct and view the urdf tree
* use `display.launch` to visualize the links (remember to modify the urdf file name in the launch file)

### Create a new urdf file for gazebo simulation

Copy the `create_urdf_for_gazebo.py` script under the `bipedal_robot_example/unitree_h1/h1_description/script/` to your package's `script` folder. Modify the script to match the urdf file of your robot. And then run: 
```bash
# in the package path, change the arguments to match your robot
python3 script/create_urdf_for_gazebo.py --urdf_in urdf/xxx.urdf --urdf_out urdf/xxx_gazebo.urdf --root_link {base_link_name}
```

this script will create a new urdf that is suitable for gazebo simulation. 

Some tips: 

- You can check the base link name of your robot in your urdf file, or use `check_urdf` to view the urdf tree. The base link name is used in the gazebo plugin `libgazebo_ros_p3d` to get the ground truth pose of the robot.
- If your robot does not have a imu link, the script will ask you to tpye in the link name and corresponding joint name of the imu. You can name them as you like. 
- You can use vscode to format the output urdf file to make it more readable.


## 4 Set up Gazebo Simulation

### Config ros control for gazebo

In our framework, use create a new hardware interface for Gazebo in `bipedal_gazebo` to replace the `DefaultRobotHWSim`. 

Under your robot package's `config` folder, create a new yaml file to config the hardware interface. For example, you can create a file named `hw_gazebo.yaml`. Refer to `bipedal_robot_example/openloong_description/config/hw_gazebo.yaml` for an example. Change the content to match your robot.

The gazebo ros control plugin has been set by the `create_urdf_for_gazebo.py` script. You can check the `gazebo_ros_control` tag in the urdf file to see if the plugin is correctly set.

### Create a launch file to start gazebo simulation

Under your robot package's `launch` folder, create a new launch file to start the gazebo simulation. For example, you can create a file named `gazebo.launch`. Refer to `bipedal_robot_example/unitree_h1/h1_description/launch/gazebo.launch` for an example. Change the content to match your robot.

----------------------------------------------------------------------
----------------------------------------------------------------------

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