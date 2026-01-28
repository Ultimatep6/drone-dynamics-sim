from __future__ import annotations

from typing import Type

import numpy as np

from quad_sim.math.rotations import _get_body_to_inertial


# TODO: Add flags for 'position','velocity','acceleration','velocity','force','moment' to then use for converting
class EarthFixed:
    # Note that vec is the vecition of the drone COG
    def __init__(
        self,
        X: float | int | np.integer | np.floating,
        Y: float | int | np.integer | np.floating,
        Z: float | int | np.integer | np.floating,
        flag: str = "position",
    ):
        # We initiate the frame of reference with orthogonal basis vectors
        self.__basisX = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        self.__basisY = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        self.__basisZ = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        self.__flag_list = [
            "position",
            "velocity",
            "acceleration",
            "ang_velocity",
            "ang_acceleration",
            "force",
            "moment",
        ]

        self.flag = flag

        self.vec = [X, Y, Z]

    @classmethod
    def from_BodyFixed(
        cls, objB: object, quaternion: np.ndarray, CM: object = None, flag: str = ""
    ):
        """
        A constructor that converts an BodyFixed vector into a EarthFixed vector
        """

        if objB.__class__.__name__ != "BodyFixed":
            raise TypeError("obj_B must be an BodyFixed object")

        rotation_matrix = _get_body_to_inertial(quaternion)

        if objB.flag == "position":
            if CM.__class__.__name__ != "EarthFixed":
                raise TypeError("CM must be an EarthFixed object")

            vec_EF = CM.vec + rotation_matrix @ objB.vec  # pyright: ignore[reportAttributeAccessIssue]
        else:
            vec_EF = rotation_matrix @ objB.vec  # pyright: ignore[reportAttributeAccessIssue]

        if flag != "":
            return cls.from_Array(vec_EF, flag=flag)
        return cls.from_Array(vec_EF, flag=objB.flag)

    @classmethod
    def from_Array(cls, arr: np.ndarray, flag: str = "position"):
        flag_list = [
            "position",
            "velocity",
            "acceleration",
            "ang_velocity",
            "ang_acceleration",
            "force",
            "moment",
        ]

        if not isinstance(arr, np.ndarray):
            raise TypeError("arr must be a np.ndarray")

        if not isinstance(flag, str):
            raise TypeError("flag must be a string")

        if flag not in flag_list:
            raise ValueError(f"flag must be one of {' or '.join(flag_list)}")

        return cls(*arr.ravel(), flag=flag)

    def __add__(self, other: EarthFixed | np.ndarray) -> EarthFixed:
        if isinstance(other, np.ndarray):
            if other.shape != self.vec.shape:
                raise TypeError(f"ndarray must have shape {self.vec.shape}")
            return EarthFixed.from_Array(self.vec + other, flag=self.flag)

        if isinstance(other, EarthFixed):
            if self.flag != other.flag:
                raise TypeError("Cannot subtract vectors with different flags")
            return EarthFixed.from_Array(self.vec + other.vec, flag=self.flag)

        raise TypeError("Operand must be EarthFixed or ndarray")

    def __sub__(self, other: EarthFixed | np.ndarray) -> EarthFixed:
        if isinstance(other, np.ndarray):
            if other.shape != self.vec.shape:
                raise TypeError(f"ndarray must have shape {self.vec.shape}")
            return EarthFixed.from_Array(self.vec - other, flag=self.flag)

        if isinstance(other, EarthFixed):
            if self.flag != other.flag:
                raise TypeError("Cannot subtract vectors with different flags")
            return EarthFixed.from_Array(self.vec - other.vec, flag=self.flag)

        raise TypeError("Operand must be EarthFixed or ndarray")

    def __mul__(self, other: EarthFixed | int | float | np.ndarray) -> EarthFixed:
        if isinstance(other, EarthFixed):
            return EarthFixed.from_Array(
                np.cross(self.vec, other.vec, axis=0), flag=self.flag
            )
        if isinstance(other, (int, float)):
            return EarthFixed.from_Array(self.vec * other, flag=self.flag)

        if isinstance(other, np.ndarray):
            return EarthFixed.from_Array(
                np.cross(self.vec, other, axis=0), flag=self.flag
            )

        raise TypeError("Operand must be EarthFixed, int, or float")

    def __eq__(self, other) -> bool:
        if not isinstance(other, EarthFixed):
            return NotImplemented

        return self.flag == other.flag and np.allclose(self.vec, other.vec, atol=1e-12)

    @property
    def vec(self):
        return self._vec

    @vec.setter
    def vec(self, value):
        # Expect a tuple/list of three scalars
        if not (isinstance(value, (tuple, list)) and len(value) == 3):
            raise TypeError(
                "vec must be a tuple/list of three numeric values (x, y, z)"
            )

        x, y, z = value

        # Validate scalar types
        for v, name in zip((x, y, z), ("x", "y", "z")):
            if not isinstance(v, (int, float, np.integer, np.floating)):
                raise TypeError(f"{name} must be a numeric scalar")

        # Build the canonical (3,1) float32 array
        arr = np.array([[x], [y], [z]], dtype=np.float32)

        self._vec = arr

    @property
    def flag(self):
        return self._flag

    @flag.setter
    def flag(self, value: str):
        if not isinstance(value, str):
            raise TypeError("flag must be a str")
        elif value not in self.__flag_list:
            raise ValueError(f"flag must be in {' '.join(self.__flag_list)}")
        else:
            self._flag = value
