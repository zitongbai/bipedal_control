#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from std_srvs.srv import Empty
import time
import argparse

def get_params():
    parser = argparse.ArgumentParser(description='switch joint init pos controller')
    parser.add_argument('--start', action='store_true', help='start the joint init pos controller')
    return parser.parse_args()

def switch_joint_init_pos_controller():
    
    # get params
    args = get_params()
    start = args.start
    
    
    rospy.init_node('joint_init_controller_switcher', anonymous=True)
    rospy.wait_for_service('/controller_manager/switch_controller')
    switch_controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    
    try:
        
        # -------------------------------------------------------------------
        # Then start the initial joint position controller
        # -------------------------------------------------------------------
        switch_controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        
        # Create a SwitchController request
        req = SwitchControllerRequest()
        
        if start:
            req.start_controllers = ['controllers/init_joint_pos_controller']  # List of controllers to start
            req.stop_controllers = ['']  # List of controllers to stop
        else:
            req.start_controllers = ['']
            req.stop_controllers = ['controllers/init_joint_pos_controller']
        req.strictness = 1
        req.start_asap = False
        req.timeout = 0.0
        
        # Call the service
        response = switch_controller_service(req)
        if response.ok:
            rospy.loginfo('Controller switch successful')
        else:
            rospy.logwarn('Controller switch failed')
        
        time.sleep(2.0)
        
    
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))

if __name__ == '__main__':
    switch_joint_init_pos_controller()