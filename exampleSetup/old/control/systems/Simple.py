import numpy as np

from quad_sim.dynamics.quadcopter import Quadcopter

#BUG: Not sure what this is
class Simple():
    """
    A class of controller for manually controlling the drones
    """

    def __init__(self, body: Quadcopter) -> None:

    def compute_controller(self):
        """
        Returns a vector of desired Thrust,Yaw_Rate,Pitch_Rate,Roll_Rate
        """
        # State Vector of inputs

        # Convert into desired values based on physical limits
        outputs = [
            self._controller_to_rate(
                i,
                self._inputRanges[i][0],
                self._inputRanges[i][1],
                self._outputRanges[i][0],
                self._outputRanges[i][1],
            )
            for i in range(self.stick_inputs.shape[0])
        ]
        print(outputs)
        # Ensure to clip the different moments to their respective possible theoretical values
        Mx_min, Mx_max, My_min, My_max, Mz_min, Mz_max = (
            self.body.max_moments_for_thrust(outputs[0])
        )

        print(f"Yaw ranges(z) : ({Mz_min},{Mz_max})")
        print(f"Pitch ranges(y) : ({My_min},{My_max})")
        print(f"Roll ranges(x) : ({Mx_min},{Mx_max})")

        # Yaw
        outputs[1] = np.clip(outputs[1], Mz_min, Mz_max)
        # Pitch
        outputs[2] = np.clip(outputs[2], My_min, My_max)
        # Roll
        outputs[3] = np.clip(outputs[3], Mx_min, Mx_max)

        return outputs

    def angle_controller(joystick_input: float, body: Quadcopter, axis=0):
        I = body.inertia_tensor[axis, axis]
        GAIN = I / 0.3

        # Convert joystick to rate
        rate_des = controller_to_rate(joystick_input)
        er_rate = rate_des - body.omega.vec[axis, 0]

        moment = np.clip(
            GAIN * er_rate, body.minMoment.vec[axis, 0], body.maxMoment.vec[axis, 0]
        )

        vec = np.zeros((3, 1))
        vec[axis, 0] = moment

        # omegas = np.sqrt(solveOmegas(body, body.mass * 9.81, vec))

    def thrust_controller(joystick_input: float, body: Quadcopter):
        if joystick_input < -1 or joystick_input > 1:
            raise ValueError("joystick_input is bound by [1,-1]")

        CLIMB_RATE = 3
        # /s
        VEL_ACC_GAIN = 1

        uT = controller_to_rate(
            joystick_input, max=1, min=-1, ratemin=-100, ratemax=100.0
        )

        z_Vel_desired = -CLIMB_RATE * uT
        vel_z = EarthFixed.from_BodyFixed(body.velocity, body.quaternion).vec[2, 0]

        print(f"The current z velocity is {vel_z}")

        velErr = z_Vel_desired - vel_z

        a_des = VEL_ACC_GAIN * velErr

        rot = _get_body_to_inertial(body.quaternion)
        print(rot.shape, body.velocity._basisZ.shape)

        b = rot @ body.velocity._basisZ.T
        z = np.array([[0], [0], [1]])

        Tcmd = (body.mass * 9.81 + a_des) / (np.dot(z.T, b))

        return Tcmd
