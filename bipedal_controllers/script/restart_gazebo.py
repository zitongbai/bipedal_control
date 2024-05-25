#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from std_srvs.srv import Empty



def restart_gazebo():
    rospy.init_node('controller_switcher', anonymous=True)
    
    rospy.wait_for_service('/controller_manager/switch_controller')
    rospy.wait_for_service('/gazebo/reset_simulation')
    
    try:
        # -------------------------------------------------------------------
        # First we should reset the gazebo simulation
        # -------------------------------------------------------------------
        reset_simulation_service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_simulation_response = reset_simulation_service()
        if reset_simulation_response:
            rospy.loginfo('Gazebo simulation reset successful')
        else:
            rospy.logwarn('Gazebo simulation reset failed')
                    
        # -------------------------------------------------------------------
        # Then we should restart the bipedal_controller
        # -------------------------------------------------------------------
        switch_controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        
        # Create a SwitchController request
        req = SwitchControllerRequest()
        req.start_controllers = ['controllers/bipedal_controller']  # List of controllers to start
        req.stop_controllers = ['']  # List of controllers to stop
        req.strictness = 0
        req.start_asap = False
        req.timeout = 0.0
        
        # Call the service
        response = switch_controller_service(req)
        
        if response.ok:
            rospy.loginfo('Controller switch successful')
        else:
            rospy.logwarn('Controller switch failed')
            
        
    
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))

if __name__ == '__main__':
    restart_gazebo()