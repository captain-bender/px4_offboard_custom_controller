#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry

class DroneController:
    def __init__(self):
        rospy.init_node("drone_controller")

        # Publishers
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/mavros/setpoint_velocity/cmd_vel", TwistStamped, self.vel_cb)

        # Services
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Variables
        self.current_state = State()
        self.current_pose = None
        self.last_velocity_time = rospy.Time.now()
        self.control_mode = "position"  # Start in position control mode
        self.timeout = rospy.Duration(1.0)  # Timeout for switching modes

        # Takeoff parameters
        self.takeoff_altitude = 1.0  # Desired takeoff altitude

        self.locked_position = None  # To store the locked position setpoint

        # Rate
        self.rate = rospy.Rate(20)

    def state_cb(self, msg):
        self.current_state = msg

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    def vel_cb(self, msg):
        # Update last velocity command time
        self.last_velocity_time = rospy.Time.now()
        if self.control_mode != "velocity":
            rospy.loginfo("Switching to velocity control")
            self.control_mode = "velocity"
    
    def lock_position(self):
        """Lock the current position as the setpoint."""
        if self.current_pose:
            self.locked_position = PoseStamped()
            self.locked_position.header.stamp = rospy.Time.now()
            self.locked_position.pose.position.x = self.current_pose.position.x
            self.locked_position.pose.position.y = self.current_pose.position.y
            self.locked_position.pose.position.z = max(self.current_pose.position.z, 1.0)  # Maintain altitude
            self.locked_position.pose.orientation = self.current_pose.orientation
            rospy.loginfo(f"Position locked at: x={self.locked_position.pose.position.x}, "
                          f"y={self.locked_position.pose.position.y}, z={self.locked_position.pose.position.z}")

    def arm_and_offboard(self):
        """Arm the drone and set it to OFFBOARD mode."""
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("Waiting for FCU connection...")
            self.rate.sleep()

        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = self.takeoff_altitude

        # Publish a few setpoints before starting OFFBOARD mode
        for _ in range(100):
            if rospy.is_shutdown():
                return False
            self.pos_pub.publish(pose)
            self.rate.sleep()

        # Set OFFBOARD mode
        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = "OFFBOARD"
        
        arm_cmd = CommandBool()
        arm_cmd.value = True

        last_request = rospy.Time.now()

        while not rospy.is_shutdown():
            if (self.current_state.mode != "OFFBOARD" and 
                (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                if self.set_mode_service(custom_mode="OFFBOARD").mode_sent:
                    rospy.loginfo("OFFBOARD enabled")
                last_request = rospy.Time.now()

            elif (not self.current_state.armed and 
                  (rospy.Time.now() - last_request > rospy.Duration(5.0))):
                if self.arm_service(True).success:
                    rospy.loginfo("Vehicle armed")
                last_request = rospy.Time.now()

            # Break loop when armed and in OFFBOARD mode
            if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
                rospy.loginfo("Vehicle is armed and in OFFBOARD mode")
                return True

            # Continue publishing setpoints to maintain OFFBOARD mode
            self.pos_pub.publish(pose)
            self.rate.sleep()
        
        rospy.logerr("Failed to arm or switch to OFFBOARD mode")
        return False

    def takeoff(self):
        """Take off to a specified altitude."""
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = self.takeoff_altitude

        rospy.loginfo(f"Taking off to {self.takeoff_altitude} meters...")
        
         # Publish setpoints and monitor altitude
        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.logwarn("Waiting for local position data...")
                self.rate.sleep()
                continue

            current_altitude = self.current_pose.position.z
            rospy.loginfo_throttle(1, f"Current Altitude: {current_altitude:.2f} m")

            if current_altitude >= self.takeoff_altitude * 0.95:  # Allow a small tolerance
                rospy.loginfo("Target altitude reached!")
                self.lock_position()
                return True


            # Publish setpoint for takeoff
            self.pos_pub.publish(pose)
            self.rate.sleep()
        
        rospy.logerr("Takeoff interrupted or failed")
        return False
        
    def run(self):
        """Main control loop."""
        
        # Arm and set OFFBOARD mode before takeoff
        if not self.arm_and_offboard():
            return
        
        # Perform takeoff
        if not self.takeoff():
            return

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self.control_mode == "velocity":
                # Check for timeout to switch back to position control
                if now - self.last_velocity_time > self.timeout:
                    rospy.loginfo("Switching to position control")
                    self.control_mode = "position"
                    # Lock the current position as the setpoint
                    self.lock_position()

            if self.control_mode == "position" and self.current_pose:
                # Publish current position as setpoint for hovering
                # pose_msg = PoseStamped()
                # pose_msg.header.stamp = now
                # pose_msg.pose.position.x = self.current_pose.position.x
                # pose_msg.pose.position.y = self.current_pose.position.y
                # pose_msg.pose.position.z = max(self.current_pose.position.z, 1.0)  # Maintain altitude
                # pose_msg.pose.orientation = self.current_pose.orientation
                # self.pos_pub.publish(pose_msg)
                self.pos_pub.publish(self.locked_position)


            self.rate.sleep()


if __name__ == "__main__":
    controller = DroneController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass