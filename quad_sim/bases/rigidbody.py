from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True) # frozen=True makes it immutable, which is good for physical constants
class RigidBody:
    """
    A simple data container for the physical properties of a rigid body.
    """
    mass: float
    inertia_tensor: np.ndarray

    def __post_init__(self):
        """Pydantic-like validation after initialization."""
        if self.mass <= 0:
            raise ValueError("Mass must be positive.")
        if self.inertia_tensor.shape != (3, 3):
            raise ValueError("Inertia tensor must be a 3x3 numpy array.")
    