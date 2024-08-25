#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
import time

class Drone:
    def __init__(self):
        rospy.init_node('mavros_tester')
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.current_state = State()

    def state_callback(self, msg):
        self.current_state = msg

    def change_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_service(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"Mode set to {mode}")
                return response
            else:
                rospy.logwarn(f"Failed to set mode to {mode}")
                return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arm_service(value=True)
            if response.success:
                rospy.loginfo("Drone armed successfully.")
                return response
            else:
                rospy.logwarn("Failed to arm drone.")
                return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def disarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arm_service(value=False)
            if response.success:
                rospy.loginfo("Drone disarmed successfully.")
                return response
            else:
                rospy.logwarn("Failed to disarm drone.")
                return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

def main():
    drone = Drone()
    
    rospy.sleep(0.1)  # Wait for some time to make sure everything is initialized
    
    rospy.loginfo("Changing mode to GUIDED...")
    response = drone.change_mode("GUIDED")
    if not response or not response.mode_sent:
        rospy.logerr("Failed to change mode to GUIDED.")
        return

    rospy.loginfo("Arming drone...")
    response = drone.arm()
    if not response or not response.success:
        rospy.logerr("Failed to arm drone.")
        return

    time.sleep(10.0)

    rospy.loginfo("Disarming drone...")
    response = drone.disarm()
    if not response or not response.success:
        rospy.logerr("Failed to disarm drone.")
        return

    rospy.spin()

if __name__ == '__main__':
    main()
