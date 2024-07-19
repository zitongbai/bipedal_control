import xml.etree.ElementTree as ET
import argparse


args = argparse.ArgumentParser()
args.add_argument('--urdf', type=str, default = '../urdf/h1.urdf', help='path to the urdf file')
args.add_argument('--root_link', type=str, help='root link of the robot', required=True)

urdf_file = args.parse_args().urdf
root_link = args.parse_args().root_link

with open(urdf_file, 'r') as file:
    data = file.read()

root = ET.fromstring(data)

# ###############################################################################################
# create transmission tags for the revolute joints
# ###############################################################################################

# find all the joints of type revolute
joints = root.findall('joint')
revolute_joints_dict = {}
# save the joint name and effort, velocity limits
# check if the joint has dynamics tag, if not, add it
for joint in joints:
    joint_type = joint.get('type')
    if joint_type == 'revolute':
        name = joint.get('name')
        limit = joint.find('limit')
        effort = limit.get('effort')
        velocity = limit.get('velocity')
        revolute_joints_dict[name] = {'effort': effort, 'velocity': velocity}
        
        dynamics = joint.find('dynamics')
        if dynamics is None:
            dynamics = ET.SubElement(joint, 'dynamics')
            dynamics.set('damping', '0.0')
            dynamics.set('friction', '0.2')
        
# create transmission tags in the following format
"""
  <transmission name="joint_name_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_name">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="joint_name_motor">
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
      	<maxVelocity>velocity</maxVelocity>
	    <maxEffort>effort</maxEffort>
    </actuator>
  </transmission>
"""
for joint_name, values in revolute_joints_dict.items():
    velocity = values['velocity']
    effort = values['effort']
    transmission = ET.Element('transmission')
    transmission.set('name', joint_name+'_trans')
    
    type = ET.SubElement(transmission, 'type')
    type.text = 'transmission_interface/SimpleTransmission'
    
    joint = ET.SubElement(transmission, 'joint')
    joint.set('name', joint_name)
    hardwareInterface = ET.SubElement(joint, 'hardwareInterface')
    hardwareInterface.text = 'hardware_interface/EffortJointInterface'
    
    actuator = ET.SubElement(transmission, 'actuator')
    actuator.set('name', joint_name + '_motor')
    hardwareInterface = ET.SubElement(actuator, 'hardwareInterface')
    hardwareInterface.text = 'hardware_interface/EffortJointInterface'
    mechanicalReduction = ET.SubElement(actuator, 'mechanicalReduction')
    mechanicalReduction.text = '1'
    maxVelocity = ET.SubElement(actuator, 'maxVelocity')
    maxVelocity.text = velocity
    maxEffort = ET.SubElement(actuator, 'maxEffort')
    maxEffort.text = effort
    
    root.append(transmission)
    
# ###############################################################################################
# IMU
# ###############################################################################################

# find if there is a link with name containing 'imu'
links = root.findall('link')
imu_link = None
for link in links:
    name = link.get('name')
    if 'imu' in name:
        imu_link = link
        break
if imu_link is None:
  raise ValueError('No link with name containing imu found')

imu_link_name = imu_link.get('name')
# check if imu_link has inertial tag, if not, add it
inertial = imu_link.find('inertial')
if inertial is None:
    inertial = ET.SubElement(imu_link, 'inertial')
    mass = ET.SubElement(inertial, 'mass')
    mass.set('value', '0.01')
    inertia = ET.SubElement(inertial, 'inertia')
    inertia.set('ixx', '0.000001')
    inertia.set('ixy', '0')
    inertia.set('ixz', '0')
    inertia.set('iyy', '0.000001')
    inertia.set('iyz', '0')
    inertia.set('izz', '0.000001')
    origin = ET.SubElement(inertial, 'origin')
    origin.set('xyz', '0 0 0')
    origin.set('rpy', '0 0 0')
# find if there is a joint with name containing 'imu'
imu_joint = None
for joint in joints:
    name = joint.get('name')
    if 'imu' in name:
        imu_joint = joint
        break
if imu_joint is None:
  raise ValueError('No joint with name containing imu found')
imu_joint_name = imu_joint.get('name')
# add the following tag
"""
  <gazebo reference="${imu_name}_joint">
    <disableFixedJointLumping>true</disableFixedJointLumping>
  </gazebo>
"""
gazebo_ref = ET.Element('gazebo')
gazebo_ref.set('reference', imu_joint_name)
disableFixedJointLumping = ET.SubElement(gazebo_ref, 'disableFixedJointLumping')
disableFixedJointLumping.text = 'true'
root.append(gazebo_ref)


# ###############################################################################################
# add gazebo pulgins
# ###############################################################################################
gazebo = ET.Element('gazebo')
ros_control_plugin = ET.SubElement(gazebo, 'plugin')
ros_control_plugin.set('name', 'gazebo_ros_control')
ros_control_plugin.set('filename', 'libbipedal_hw_sim.so')
robot_name_space = ET.SubElement(ros_control_plugin, 'robotNamespace')
robot_name_space.text = '/'
robot_param = ET.SubElement(ros_control_plugin, 'robotParam')
robot_param.text = 'bipedal_robot_description_sim'
robot_sim_type = ET.SubElement(ros_control_plugin, 'robotSimType')
robot_sim_type.text = 'bipedal_gazebo/BipedalHWSim'

p3d_plugin = ET.SubElement(gazebo, 'plugin')
p3d_plugin.set('name', 'gazebo_ros_p3d')
p3d_plugin.set('filename', 'libgazebo_ros_p3d.so')
always_on = ET.SubElement(p3d_plugin, 'alwaysOn')
always_on.text = 'true'
update_rate = ET.SubElement(p3d_plugin, 'updateRate')
update_rate.text = '1000'
body_name = ET.SubElement(p3d_plugin, 'bodyName')
body_name.text = root_link
topic_name = ET.SubElement(p3d_plugin, 'topicName')
topic_name.text = 'ground_truth/state'
gaussian_noise = ET.SubElement(p3d_plugin, 'gaussianNoise')
gaussian_noise.text = '0.0'
frame_name = ET.SubElement(p3d_plugin, 'frameName')
frame_name.text = 'world'
xyz_offset = ET.SubElement(p3d_plugin, 'xyzOffsets')
xyz_offset.text = '0 0 0'
rpy_offset = ET.SubElement(p3d_plugin, 'rpyOffsets')
rpy_offset.text = '0 0 0'

root.append(gazebo)

    
# ###############################################################################################
# export the file
# ###############################################################################################
    
tree = ET.ElementTree(root)

# add the following tag in the begining of the file
# <?xml version="1.0" encoding="utf-8"?>

tree.write('../urdf/h1_gazebo.urdf', encoding='utf-8', xml_declaration=True)  
