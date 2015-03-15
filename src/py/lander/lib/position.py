#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import rospy

import geometry_msgs.msg

from mavros.srv import StreamRate, StreamRateRequest


class PositionMixin:
    def __init__(self, stream_rate_hz=50):
        # TODO: Select a more suitable initial value
        self.position = None
        self.orientation = None

        # Request higher telemetry stream rate
        set_stream_rate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)
        set_stream_rate(StreamRateRequest.STREAM_POSITION, stream_rate_hz, True)

        # Would be nice to use /mavros/position/local here, but ArduCopter doesn't
        # transmit the required POSITION_LOCAL_NED messages. Instead, mavros converts
        # the global coordinates passed via the POSITION_GLOBAL_INT message into UTM
        # coordinates, which can be used for local navigation assuming the drone
        # doesn't cross zone boundaries.
        rospy.Subscriber("/mavros/global_position/local",
                geometry_msgs.msg.PoseWithCovarianceStamped,
                self.handle_position_message)

        rospy.Subscriber("/mavros/global_position/gp_vel",
            geometry_msgs.msg.Vector3Stamped,
            self.handle_velocity_message)

    def handle_position_message(self, msg):
        """
        Handle mavros/global_position/local messages.
        """
        pose = msg.pose.pose
        self.position = pose.position
        self.orientation = pose.orientation

    def handle_velocity_message(self, msg):
        """
        Handle mavros/global_position/gp_vel messages.
        """
        self.velocity = msg.vector
