from pydantic import BaseModel, Field

from quad_sim.control.PID import PID


class PidGains(BaseModel):
    kp: float = 1.0
    ki: float = 1.0
    kd: float = 1.0
    i_limit: float = 0.0
    out_limit: float = 0.5
    alpha: float = 0.5
    dt: float = 0.1


class GainsConf(BaseModel):
    # Angle → Rate mapping (outer loop)
    roll: PidGains = Field(
        default_factory=lambda: PidGains(
            kp=6.0, ki=0.0, kd=0.0, out_limit=4.5, i_limit=0.0, alpha=1.0
        )
    )
    pitch: PidGains = Field(
        default_factory=lambda: PidGains(
            kp=6.0, ki=0.0, kd=0.0, out_limit=4.5, i_limit=0.0, alpha=1.0
        )
    )
    yaw: PidGains = Field(
        default_factory=lambda: PidGains(
            kp=4.0, ki=0.0, kd=0.0, out_limit=3.0, i_limit=0.0, alpha=1.0
        )
    )

    # Rate → Moments (inner loop)
    roll_rate: PidGains = Field(
        default_factory=lambda: PidGains(
            kp=0.12, ki=0.08, kd=0.004, out_limit=0.4, i_limit=0.15, alpha=0.3
        )
    )
    pitch_rate: PidGains = Field(
        default_factory=lambda: PidGains(
            kp=0.12, ki=0.08, kd=0.004, out_limit=0.4, i_limit=0.15, alpha=0.3
        )
    )
    yaw_rate: PidGains = Field(
        default_factory=lambda: PidGains(
            kp=0.25, ki=0.10, kd=0.000, out_limit=0.3, i_limit=0.10, alpha=1.0
        )
    )

    # Velocity loop
    vx: PidGains = Field(default_factory=PidGains)
    vy: PidGains = Field(default_factory=PidGains)
    vz: PidGains = Field(
        default_factory=lambda: PidGains(
            kp=0.3, ki=0.00, kd=0.0, out_limit=1.0, i_limit=0.3, alpha=0.2
        )
    )

    # Position loop
    px: PidGains = Field(default_factory=PidGains)
    py: PidGains = Field(default_factory=PidGains)
    pz: PidGains = Field(default_factory=PidGains)

    controllers: dict = Field(default_factory=dict)

    def model_post_init(self, __context):
        self.controllers = {
            name: PID(gains)
            for name, gains in self.__dict__.items()
            if isinstance(gains, PidGains)
        }
