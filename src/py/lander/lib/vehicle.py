import rospy

import geometry_msgs.msg

from lander.lib.position import PositionMixin


class Vehicle(object, PositionMixin):
    """
    Simple model of an aerial vehicle.
    """
    def __init__(self):
        PositionMixin.__init__(self)

        self.location_setpoint_publisher = \
                rospy.Publisher("/mavros/setpoint/local_position",
                        geometry_msgs.msg.PoseStamped,
                        queue_size=10)

        self.velocity_setpoint_publisher = \
                rospy.Publisher("/mavros/setpoint/cmd_vel",
                        geometry_msgs.msg.TwistStamped,
                        queue_size=10)

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
