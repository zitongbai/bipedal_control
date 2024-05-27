#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from std_srvs.srv import Empty
import time


def restart_gazebo():
    rospy.init_node('controller_switcher', anonymous=True)
    
    rospy.wait_for_service('/controller_manager/switch_controller')
    rospy.wait_for_service('/gazebo/reset_simulation')
    
    reset_simulation_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    switch_controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    
    try:
        
        # -------------------------------------------------------------------
        # Then start the initial joint position controller
        # -------------------------------------------------------------------
        switch_controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        
        # Create a SwitchController request
        req = SwitchControllerRequest()
        req.start_controllers = ['controllers/init_joint_pos_controller']  # List of controllers to start
        req.stop_controllers = ['controllers/bipedal_controller']  # List of controllers to stop
        req.strictness = 1
        req.start_asap = False
        req.timeout = 0.0
        
        # Call the service
        response = switch_controller_service(req)
        if response.ok:
            rospy.loginfo('Controller switch 1 successful')
        else:
            rospy.logwarn('Controller switch 1 failed')
        
        time.sleep(2.0)
        
        # -------------------------------------------------------------------
        # First we should reset the gazebo simulation
        # -------------------------------------------------------------------
        reset_simulation_response = reset_simulation_service()
        if reset_simulation_response:
            rospy.loginfo('Gazebo simulation reset successful')
        else:
            rospy.logwarn('Gazebo simulation reset failed')
          
            
        # -------------------------------------------------------------------
        # Then switch to bipedal controller
        # -------------------------------------------------------------------
        time.sleep(0.5)
        req.start_controllers = ['controllers/bipedal_controller']  # List of controllers to start
        req.stop_controllers = ['controllers/init_joint_pos_controller']  # List of controllers to stop
        
        # Call the service
        response = switch_controller_service(req)
        if response.ok:
            rospy.loginfo('Controller switch 2 successful')
        else:
            rospy.logwarn('Controller switch 2 failed')
        
    
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))

if __name__ == '__main__':
    restart_gazebo()