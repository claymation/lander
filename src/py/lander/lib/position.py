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
        rospy.wait_for_service("mavros/set_stream_rate")
        set_stream_rate = rospy.ServiceProxy("mavros/set_stream_rate", StreamRate)
        set_stream_rate(StreamRateRequest.STREAM_POSITION, stream_rate_hz, True)

        rospy.Subscriber("/mavros/local_position/local",
                geometry_msgs.msg.PoseStamped,
                self.handle_position_message)

        rospy.Subscriber("/mavros/global_position/gp_vel",
            geometry_msgs.msg.Vector3Stamped,
            self.handle_velocity_message)

    def handle_position_message(self, msg):
        """
        Handle mavros/local_position/local messages.
        """
        pose = msg.pose
        self.position = pose.position
        self.orientation = pose.orientation

    def handle_velocity_message(self, msg):
        """
        Handle mavros/global_position/gp_vel messages.
        """
        self.velocity = msg.vector
