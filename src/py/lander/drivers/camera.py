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

    def release(self):
        pass

    def back_project(self, u, v):
        """
        Project a point from (u, v) pixel coordinates to relative (dx, dy, dz)
        world coordinates, assuming the point lies on the ground plane.
        """
        # Get euler angles from vehicle pose
        o = self.orientation
        q = [o.x, o.y, o.z, o.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        # Compute rotation matrix:
        #   - apply rigid body rotations around body axes, in yaw-pitch-roll order,
        #     to compute the camera's pose in world coordinates
        # NB: mavros negates yaw and pitch (why?!), so we correct for that here
        R = numpy.matrix(euler_matrix(-yaw, -pitch, roll, axes="rzxy"))

        # Express image point as a column vector in homogeneous coordinates
        P_i = numpy.matrix((u, v, 1)).T

        # Back-project and rotate a point in image coords to a ray in camera coords
        K = self.P[:, :3]
        R = R[:3, :3]
        P_c = R * K.I * P_i

        # Recover 3D point (in camera coordinates) from homogeneous coordinates
        # TODO: Compute the point's depth in the scene, based on camera position
        #       and orientation. For now, we assume the vehicle is mostly level,
        #       so we use the (estimated) AGL altitude to estimate the depth.
        d = self.position.z
        P_c = P_c / P_c[2] * d

        return P_c


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
        # assuming its right in the center of the frame
        self.frame_size = (self.P[0,2] * 2, self.P[1,2] * 2)

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

        # Convert from right-handed to left-handed coordinate system
        # (assuming that the camera points down, such that positive z is down)
        L = numpy.diag([1, 1, -1, 1])

        # Compute rotation matrix:
        #   - apply rigid body rotations around body axes, in yaw-pitch-roll order,
        #     to compute the camera's pose in world coordinates
        #   - take the inverse (transpose) rotation matrix to map from world
        #     to camera coordinates
        # NB: Not sure why the angles need to be negated here, but mavros already
        #     negates yaw and pitch, so we only need to negate roll (but why?!)
        R = numpy.matrix(euler_matrix(yaw, pitch, -roll, axes="rzxy")).T

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
