#!/bin/python

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from std_srvs.srv import Empty
import time
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mode_schedule


def restart_gazebo():
    rospy.init_node('controller_switcher', anonymous=True)
    
    rospy.wait_for_service('/controller_manager/switch_controller')
    rospy.wait_for_service('/gazebo/reset_simulation')
    
    reset_simulation_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    switch_controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    
    robot_name = "bipedal_robot"
    gait_pub = rospy.Publisher(robot_name+"_mpc_mode_schedule", mode_schedule, queue_size=1)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    try:
        
        # -------------------------------------------------------------------
        # Then start the initial joint position controller
        # -------------------------------------------------------------------
        switch_controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        
        # Create a SwitchController request
        req = SwitchControllerRequest()
        req.start_controllers = ['controllers/init_joint_pos_controller', 'controllers/upper_joint_controller']  # List of controllers to start
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
        
        time.sleep(3.0)
        
        # -------------------------------------------------------------------
        # First we should reset the gazebo simulation
        # -------------------------------------------------------------------
        reset_simulation_response = reset_simulation_service()
        if reset_simulation_response:
            rospy.loginfo('Gazebo simulation reset successful')
        else:
            rospy.logwarn('Gazebo simulation reset failed')
          
        # input('Press Enter to continue...')
        # -------------------------------------------------------------------
        # Then switch to bipedal controller
        # -------------------------------------------------------------------
        time.sleep(0.05)
        req.start_controllers = ['controllers/bipedal_controller']  # List of controllers to start
        req.stop_controllers = ['controllers/init_joint_pos_controller']  # List of controllers to stop
        
        # Call the service
        response = switch_controller_service(req)
        if response.ok:
            rospy.loginfo('Controller switch 2 successful')
        else:
            rospy.logwarn('Controller switch 2 failed')
        
        # time.sleep(1.0)
        # # publish gait
        # # refer to gait.info
        # gait_msg = mode_schedule()
        # # stance
        # #  {
        # #    modeSequence
        # #    {
        # #      [0]     STANCE
        # #    }
        # #    switchingTimes
        # #    {
        # #      [0]     0.0
        # #      [1]     0.5
        # #    }
        # # }
        # gait_msg.eventTimes = [0.0, 0.5]
        # gait_msg.modeSequence = [3] # stance, refer to ocs2_bipedal_robot/gait/MotionPhaseDefinition.h
        # gait_pub.publish(gait_msg)
        # rospy.loginfo("Publish stance gait")
        
        time.sleep(0.1)
        # send the trajectory cmd_vel once
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.angular.z = 0.0
        cmd_vel_pub.publish(cmd_vel_msg)
        rospy.loginfo("Publish cmd_vel")
    
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))

if __name__ == '__main__':
    restart_gazebo()