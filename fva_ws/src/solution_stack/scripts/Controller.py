

from typing import Callable, Dict, Optional
import numpy as np




# ----- Lateral Controller ------------------
KP_STANLEY = 1e-3


def stanley_controller(car,error):
    v = car.getVel()[0]


    steer = np.arctan2(KP_STANLEY*error/v,1)
    steer = np.clip(steer,np.deg2rad(-20.0),np.deg2rad(20.0))

    
    return steer



# --- Defaults / plant params (can be overridden per-controller) ---
DEFAULTS = {
    "M": 6.3,                # mass [kg]
    "K": 1.0,                # plant gain (throttle -> acceleration)
    "RX": 0.6616134,       # rolling resistance force [N]
    "DT": 0.05,             # default controller timestep [s]
    "U_MAX": 1.0,
    "U_MIN": 0.0,
    "KBW": 0.0,
}

class Controller:
    def __init__(self, car,KP,KI,KD):
        self.car = car

        self.KP = KP
        self.KI = KI
        self.KD = KD

        # mutable state
        self.last_error = 0.0
        self.last_u = 0.0


    def reset(self):
        self.last_error = 0.0
        self.last_u = 0.0

    def control(self, ref: float) -> float:
        v = float(self.car.getVel()[0])
        error = ref - v

        delta_u = self.KP * (error - self.last_error) + self.KI * DEFAULTS['DT'] * error
        u_unsat = self.last_u + delta_u

        u_sat = float(np.clip(u_unsat, DEFAULTS['U_MIN'], DEFAULTS['U_MAX']))

        # back-calculation anti-windup (applied as a correction to internal integrator)
        u_corr = u_unsat + DEFAULTS['KBW'] * (u_sat - u_unsat)

        # update states using applied (saturated) value
        self.last_error = error
        self.last_u = u_sat

        return u_sat
