from typing import Type

import numpy as np
from numpy import cos as c
from numpy import sin as s

from quad_sim.math.references.earthFixed import EarthFixed
from quad_sim.math.rotations import _get_body_to_inertial


class BodyFixed:
    # Note that pos here is the position on the rigid body (not the EarthFixed)
    def __init__(self, pos):
        self.pos = pos

        # We initiate the frame of reference with orthogonal basis vectors
        self.__basisX = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
        self.__basisY = np.array([[0.0, 1.0, 0.0]], dtype=np.float32)
        self.__basisZ = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)

        self.__X = self.__basisX @ pos
        self.__Y = self.__basisY @ pos
        self.__Z = self.__basisZ @ pos

        # Orientationa angles in the form of pitch, roll, yaw
        self.orientation = np.zeros(shape=(1, 3), dtype=np.float32)

    @classmethod
    def from_EarthFixed(cls, objB: EarthFixed, quaternion: np.ndarray):
        """
        A constructor that converts an EarthFixed vector into a BodyFixed vector
        """

        # TODO:
        # Find a way to remove the CM component of a given BodyFixed point

        if not isinstance(objB, EarthFixed):
            raise TypeError("obj_B must be an EarthFixed object")

        rotation_matrix = _get_body_to_inertial(quaternion)

        return cls(rotation_matrix @ objB.pos)

    def _toearthFixed(self, _earth_conv_Mat: np.ndarray) -> np.ndarray:
        """
        Converts a point on the quadcopter (ex. motor) given in the BodyFixed frame to the
        EarthFixed frame as a point in space
        """

        if not isinstance(_earth_conv_Mat, np.ndarray):
            raise TypeError("_earth_conv_Mat must be np.ndarray")

        elif _earth_conv_Mat.shape != (3, 3):
            raise TypeError("_earth_conv_Mat must be a np.ndarray of shape (3,3)")

        # Point of rigid body in EF space
        posEF = _earth_conv_Mat @ self.pos

        return posEF

    def __updateOrientation(self):
        self.__p = float(self.orientation[0, 0])
        self.__r = float(self.orientation[0, 1])
        self.__y = float(self.orientation[0, 2])

    def __updateConversionMat(self):
        # Conversion Marices
        # -> EarthFixed
        self._earth_convMat = np.array(
            [
                [
                    c(self.__y) * c(self.__p),
                    c(self.__y) * s(self.__p) * s(self.__r) - s(self.__y) * c(self.__r),
                    c(self.__y) * s(self.__p) * c(self.__r) + s(self.__y) * s(self.__r),
                ],
                [
                    s(self.__y) * c(self.__p),
                    s(self.__y) * s(self.__p) * s(self.__r) + c(self.__y) * c(self.__r),
                    s(self.__y) * s(self.__p) * c(self.__r) - c(self.__y) * s(self.__r),
                ],
                [-s(self.__p), c(self.__p) * s(self.__r), c(self.__p) * c(self.__r)],
            ]
        )

        self._earth_convMat[np.abs(self._earth_convMat) < 1e-15] = 0

    def __str__(self) -> str:
        return f"Rotations: \n\
        Pitch: {self.__p}\n\
        Roll: {self.__r}\n\
        Yaw: {self.__y}\n\
        ------------------------------\n\
        \n\
        Rotation Matrix: \n\
        Row 1 {self._earth_convMat[0, :]} \n\
        Row 2 {self._earth_convMat[1, :]} \n\
        Row 3 {self._earth_convMat[2, :]} \n\
        "

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, value: np.ndarray):
        if not isinstance(value, np.ndarray):
            raise TypeError("pos must be a  np.ndarray")

        elif value.shape != (3, 1):
            raise TypeError("pos must be np.ndarray shape (3,1)")

        else:
            self._pos = value

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value: np.ndarray):
        if not isinstance(value, np.ndarray):
            raise TypeError(np.ndarray)

        else:
            self._orientation = value
            self.__updateOrientation()
            self.__updateConversionMat()
