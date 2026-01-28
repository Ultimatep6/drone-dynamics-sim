import numpy as np


def asin(x):
    return np.arcsin(np.clip(x, -1.0, 1.0))


def acos(x):
    return np.arccos(np.clip(x, -1.0, 1.0))


def atan2(y, x):
    return np.arctan2(y, x)
