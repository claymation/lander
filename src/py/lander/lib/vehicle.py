#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import rospy

import geometry_msgs.msg

from mavros.srv import CommandBool, SetMode

from lander.lib.position import PositionMixin


class Vehicle(object, PositionMixin):
    """
    Simple model of an aerial vehicle.
    """
    def __init__(self):
        PositionMixin.__init__(self)

        self.location_setpoint_publisher = \
                rospy.Publisher("/mavros/setpoint_position/local_position",
                        geometry_msgs.msg.PoseStamped,
                        queue_size=10)

        self.velocity_setpoint_publisher = \
                rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",
                        geometry_msgs.msg.TwistStamped,
                        queue_size=10)

        self.acceleration_setpoint_publisher = \
                rospy.Publisher("/mavros/setpoint_accel/accel",
                        geometry_msgs.msg.Vector3Stamped,
                        queue_size=10)

    def set_armed_state(self, state):
        set_armed_state = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        set_armed_state(value=state)

    def arm(self):
        self.set_armed_state(True)

    def disarm(self):
        self.set_armed_state(False)

    def set_mode(self, mode):
        """
        Ask the FCU to transition to the specified custom flight mode.
        """
        set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
        set_mode(custom_mode=mode)

    def set_location_setpoint(self, setpoint):
        """
        Publish a SET_POSITION_TARGET_LOCAL_NED message with
        position x, y, z and yaw.
        """
        x, y, z, yaw = setpoint

        msg = geometry_msgs.msg.PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.z = yaw

        self.location_setpoint_publisher.publish(msg)

    def set_velocity_setpoint(self, setpoint):
        """
        Publish a SET_POSITION_TARGET_LOCAL_NED message with linear
        velocities vx, vy, vz and angular velocity yaw_rate.
        """
        vx, vy, vz, yaw_rate = setpoint

        msg = geometry_msgs.msg.TwistStamped()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate

        self.velocity_setpoint_publisher.publish(msg)

    def set_acceleration_setpoint(self, setpoint):
        """
        Publish a SET_POSITION_TARGET_LOCAL_NED message with linear
        accelerations ax, ay, az.
        """
        ax, ay, az = setpoint

        msg = geometry_msgs.msg.Vector3Stamped()
        msg.vector.x = ax
        msg.vector.y = ay
        msg.vector.z = az

        self.acceleration_setpoint_publisher.publish(msg)
