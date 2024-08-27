#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State, PositionTarget
import time

class Drone:
    def __init__(self):
        rospy.init_node('mavros_tester')
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.position_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
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

    def takeoff(self, altitude):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            response = takeoff_service(altitude=altitude)
            if response.success:
                rospy.loginfo("Takeoff successful.")
                return response
            else:
                rospy.logwarn("Failed to take off.")
                return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def set_local_position(self, x, y, z):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = 0b110111111000  # Position target mask
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        self.position_pub.publish(msg)

def main():
    drone = Drone()
    
    rospy.sleep(0.1)  # Wait for some time to make sure everything is initialized
    
    # Check if the drone is in GUIDED mode before proceeding
    if drone.current_state.mode != "GUIDED":
        rospy.logerr("Drone is not in GUIDED mode. Exiting...")
        return

    # Move to local position (x=6, y=6, z=3)
    x, y, z = 6.0, 6.0, 3.0
    rospy.loginfo(f"Moving to local position (x={x}, y={y}, z={z})...")
    drone.set_local_position(x=x, y=y, z=z)
    time.sleep(20.0)

    # Move to local position (x=6, y=0, z=3)
    x, y, z = 6.0, 0.0, 3.0
    rospy.loginfo(f"Moving to local position (x={x}, y={y}, z={z})...")
    drone.set_local_position(x=x, y=y, z=z)
    time.sleep(20.0)

    # Change mode to LAND
    rospy.loginfo("Changing mode to LAND...")
    response = drone.change_mode("LAND")
    if not response or not response.mode_sent:
        rospy.logerr("Failed to change mode to LAND.")
        return

    time.sleep(20.0)  # Wait for landing to complete

    while drone.current_state.armed:
        rospy.loginfo("Disarming drone...")
        response = drone.disarm()
        if not response or not response.success:
            rospy.logerr("Failed to disarm drone.")
        time.sleep(1.0)

    time.sleep(5.0)

    rospy.loginfo("Changing mode to GUIDED before re-arming...")
    response = drone.change_mode("GUIDED")
    if not response or not response.mode_sent:
        rospy.logerr("Failed to change mode to GUIDED.")
        return

    time.sleep(5.0)

    while not drone.current_state.armed:
        rospy.loginfo("Re-arming drone...")
        response = drone.arm()
        if not response or not response.success:
            rospy.logerr("Failed to re-arm drone.")
        time.sleep(1.0)

    time.sleep(5.0)

    rospy.loginfo("Taking off again...")
    response = drone.takeoff(altitude=3.0)
    if not response or not response.success:
        rospy.logerr("Failed to take off.")
        return

    time.sleep(10.0)

    x, y, z = 0.0, 6.0, 3.0
    rospy.loginfo(f"Moving to local position (x={x}, y={y}, z={z})...")
    drone.set_local_position(x=x, y=y, z=z)
    time.sleep(20.0)

    x, y, z = 0.0, 0.0, 3.0
    rospy.loginfo(f"Moving to local position (x={x}, y={y}, z={z})...")
    drone.set_local_position(x=x, y=y, z=z)
    time.sleep(10.0)

    rospy.loginfo("Changing mode to LAND...")
    response = drone.change_mode("LAND")
    if not response or not response.mode_sent:
        rospy.logerr("Failed to change mode to LAND.")
        return

    rospy.spin()

if __name__ == '__main__':
    main()
