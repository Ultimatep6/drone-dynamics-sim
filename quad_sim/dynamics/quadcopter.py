import numpy as np

from quad_sim.math.references.bodyFixed import BodyFixed
from quad_sim.math.references.earthFixed import EarthFixed


class Quadcopter:
    def __init__(
        self,
        mass: int,
        rotors: int = 4,
        radius: float = 0.30,
        EF_pos: np.ndarray = np.zeros((3, 1), dtype=np.float32),
    ) -> None:
        # Assume the center of mass is located at origin
        self.CM = BodyFixed(pos=np.zeros(shape=(3, 1), dtype=np.float32))

        # By default 0,0,0
        self.EF_pos = EarthFixed(pos=EF_pos)

        # kg
        self.mass = mass

        self.rotors = rotors

    def __get_rotor_positions(self) -> list[BodyFixed]:
        """
        Calculates the position of all the rotors assuming they are equidistant on the unit circle
        """

        pos = []
        for rotor in range(self.rotors):
            disp = np.array(
                [
                    [np.cos((2 * np.pi / self.rotors) * rotor)],
                    [np.sin((2 * np.pi / self.rotors) * rotor)],
                    [0],
                ]
            )

            disp[np.abs(disp) < 1e-15] = 0
            pos.append(BodyFixed(self.CM.pos + disp))
        return pos

    def __str__(self) -> str:
        rot_str = "\n".join(
            [
                f"Coptor {rotor}: {self.__rotor_positions[rotor].pos.T}"
                for rotor in range(self.rotors)
            ]
        )
        
        pos_str = f"Global CM Position {self.EF_pos.pos.T}"
        return (
            "Drone Layout: \n\
-------------------------\n"
            + rot_str +
"\n\n------------------------\n\n"
            + pos_str
        )

    @property
    def EF_pos(self):
        return self._EF_pos

    @EF_pos.setter
    def EF_pos(self, value: EarthFixed):
        if not isinstance(value, EarthFixed):
            raise TypeError("EF_pos must be a WorldFixed")
        else:
            self._EF_pos = value

    @property
    def rotors(self):
        return self._rotors

    @rotors.setter
    def rotors(self, value: int):
        if not isinstance(value, int):
            raise TypeError("rotors must be a int")
        else:
            self._rotors = value
            self.__rotor_positions = self.__get_rotor_positions()
