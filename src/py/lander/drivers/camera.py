#!/usr/bin/env python
# vim: set ts=4 sw=4 et:

import time

import cv2
import numpy
import rospy

from tf.transformations import euler_from_quaternion, euler_matrix

from lander.lib.position import PositionMixin


class Camera(object, PositionMixin):
    """
    Abstract base camera.
    """
    def __init__(self, camera_matrix):
        """
        Initialize a camera, given its 3x4 instrinsic calibration matrix.
        """
        PositionMixin.__init__(self)

        self.P = camera_matrix

        # Invert fy to account for the fact that positive y points down
        # in pixel coordinates but points up in camera coordinates
        self.P[1,1] = -self.P[1,1]

    def release(self):
        pass

    def back_project(self, u, v):
        """
        Project a point from (u, v) pixel coordinates to (x, y, z) world coordinates,
        assuming the point lies on the ground plane.
        """
        # Express image point as a column vector in homogeneous coordinates
        P_i = numpy.matrix((u, v, 1)).T

        # Project the point from image coordinates to a ray in camera coordinates
        K = self.P[:, :3]
        P_c = K.I * P_i

        # Recover 3D point from ray, using the vehicle's altitude to estimate
        # the point's depth in the scene
        d = self.position.z
        P_c = P_c / P_c[2] * d

        # Build translation vector
        p = self.position
        c = numpy.matrix((p.x, p.y, p.z)).T

        # Get euler angles from vehicle pose
        o = self.orientation
        q = [o.x, o.y, o.z, o.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        # Compute rotation matrix, applying rigid body rotations around body axes,
        # in yaw-pitch-roll order, to compute the camera's pose in world coordinates
        # NB: mavros is still working on sign conventions; in the meantime, we need
        #     to negate roll to match mavlink/mavproxy
        R = numpy.matrix(euler_matrix(yaw, pitch, -roll, axes="rzxy"))[:3,:3]

        # Convert from left-handed to right-handed coordinate system
        # (assuming that the camera points down, such that positive z is down)
        L = numpy.diag([1, 1, -1])

        # Rotate and translate the point from left-handed camera coordinates
        # to right-handed world coordinates
        P_w = (R * L * P_c) + c

        return P_w


class OpenCVCamera(Camera):
    """
    An OpenCV-compatible video capture device.
    """
    def __init__(self, *args, **kwargs):
        super(OpenCVCamera, self).__init__(*args, **kwargs)

        self.cap = cv2.VideoCapture(0)

    def release(self):
        """
        Release the video capture handle.
        """
        self.cap.release()

    def get_frame(self):
        """
        Return a frame captured by the camera.
        """
        ok, frame = self.cap.read()
        if ok:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame


class SimulatedCamera(Camera):
    """
    A simulated camera.
    """
    def __init__(self, *args, **kwargs):
        super(SimulatedCamera, self).__init__(*args, **kwargs)

        # Compute the frame size from the principal point,
        # assuming it's right in the center of the frame
        self.frame_size = (
            int(numpy.ceil(self.P[0,2]) * 2),
            int(numpy.ceil(self.P[1,2]) * 2),
        )

        # Compute the time period between frames (dt)
        self.frame_rate = rospy.get_param("~frame_rate", 30)
        self.frame_dt = 1.0 / self.frame_rate
        self.last_frame_time = 0

    def set_target(self, image_file, position, size_in_meters):
        """
        image_file: file name of target image
        position: position of the target in world coordinates ((x, y, z); meters)
        size_in_meters: size of the target ((length, width); meters)
        """
        self.target_image = cv2.cvtColor(cv2.imread(image_file), cv2.COLOR_BGR2GRAY)

        # Compute target corners in pixel (2D) coordinates (clockwise from top-left)
        size_in_pixels = self.target_image.shape
        corners = numpy.array((
            (0, 0),
            (1, 0),
            (1, 1),
            (0, 1),
        )) * size_in_pixels

        # NB: cv2.getPerspectiveTransform() requires CV_32F data type
        self.target_corners_pixel = corners.astype(numpy.float32)

        # Compute target corners in world (3D) coordinates (clockwise from top-left)
        corners = numpy.array((
            (-.5,  .5, 0),
            ( .5,  .5, 0),
            ( .5, -.5, 0),
            (-.5, -.5, 0),
        )) * size_in_meters + position

        # Convert to homogeneous coordinates (as matrix of column vectors)
        self.target_corners_world = numpy.vstack((numpy.matrix(corners.T), numpy.ones(4)))

    def get_frame(self):
        """
        Generate a simulated camera frame.
        """
        # Get euler angles from vehicle pose
        o = self.orientation
        q = [o.x, o.y, o.z, o.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        # Build translation vector
        p = self.position
        c = numpy.matrix((p.x, p.y, p.z, 1)).T

        # Fail fast if we're below the ground plane (negative altitude)
        if p.z <= 0.0:
            return numpy.zeros(self.frame_size, dtype=numpy.uint8).T

        # Convert from right-handed to left-handed coordinate system
        # (assuming that the camera points down, such that positive z is down)
        L = numpy.diag([1, 1, -1, 1])

        # Compute rotation matrix, applying rigid body rotations around body axes,
        # in yaw-pitch-roll order, to compute the camera's pose in world coordinates
        # NB: mavros is still working on sign conventions; in the meantime, we need
        #     to negate roll to match mavlink/mavproxy
        R = numpy.matrix(euler_matrix(yaw, pitch, -roll, axes="rzxy"))

        # Invert (transpose) the matrix to rotate world coordinates to camera frame
        R = R.T

        # Project target corners from right-handed world coordinates to
        # left-handed image coordinates
        corners = self.P * R * L * (self.target_corners_world - c)

        # Recover 2D points from homogeneous coordinates (as row vectors)
        corners = (corners / corners[2])[:2].T

        # NB: cv2.getPerspectiveTransform() requires CV_32F data type
        corners = corners.astype(numpy.float32)

        # Warp image
        M = cv2.getPerspectiveTransform(self.target_corners_pixel, corners)
        frame = cv2.warpPerspective(self.target_image, M, self.frame_size)

        # Simulate frame rate
        now = time.time()
        dt = now - self.last_frame_time
        rospy.sleep(self.frame_dt - dt)
        self.last_frame_time = now

        return frame
