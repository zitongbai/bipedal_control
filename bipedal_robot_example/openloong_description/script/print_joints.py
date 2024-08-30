import xml.etree.ElementTree as ET
import argparse

args = argparse.ArgumentParser()
args.add_argument('--urdf', type=str, help='path to the urdf file', required=True)

urdf_file = args.parse_args().urdf

with open(urdf_file, 'r') as file:
    data = file.read()

root = ET.fromstring(data)

# print all the revolute joints names
print('Revolute joints:')
joints = root.findall('joint')
for joint in joints:
    joint_type = joint.get('type')
    if joint_type == 'revolute':
        name = joint.get('name')
        print(name)

# print defaultJointState in reference.info
for i in range(16, 31):
    name = joints[i].get('name')
    print(f"({i-16},0)  0.00    ;   {name}")
    
# print joints in task.info
for i in range(16, 31):
    name = joints[i].get('name')
    print(f"[{i-16}] {name}")
    
# upper joints
print('Upper joints:')
for i in range(16):
    name = joints[i].get('name')
    print(f"[{i}] {name}")
    
# initialState
print('initialState:')
for i in range(16, 31):
    name = joints[i].get('name')
    print(f"({i-4},0)  0.00    ;   {name}")