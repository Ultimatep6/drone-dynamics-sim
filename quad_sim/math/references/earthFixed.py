import numpy as np


class EarthFixed:
    # Note that pos is the position of the drone COG
    def __init__(self, pos: np.ndarray):
        self.pos = pos

        # We initiate the frame of reference with orthogonal basis vectors
        self.__basisX = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        self.__basisY = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        self.__basisZ = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        self.__X = self.__basisX @ pos
        self.__Y = self.__basisY @ pos
        self.__Z = self.__basisZ @ pos

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, value: np.ndarray):
        if not isinstance(value, np.ndarray):
            raise TypeError("pos must be a  np.ndarray")
        elif value.shape != (3, 1):
            raise TypeError("pos must be a np.ndarray of shape (3,1)")
        else:
            self._pos = value
