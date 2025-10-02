

from typing import Callable, Dict, Optional
import numpy as np

# --- Defaults / plant params (can be overridden per-controller) ---
DEFAULTS = {
    "M": 6.3,                # mass [kg]
    "K": 1.0,                # plant gain (throttle -> acceleration)
    "RX": 0.6616134,       # rolling resistance force [N]
    "DT": 0.05,             # default controller timestep [s]
    "U_MAX": 1.0,
    "U_MIN": 0.0,
}

class Controller:
    def __init__(self, car,params):
        self.car = car

        
        self.parse_params(params)

        # mutable state
        self.last_error = 0.0
        self.last_u = 0.0

    def parse_params(self,params):
        # case 1: direct PID parameters
        if all(k in params for k in ("KP", "KI")):
            self.KP = params["KP"]
            self.KI = params["KI"]
            self.param_mode = "pid"

        # case 2: pole placement (ζ, ωn)
        elif all(k in params for k in ("ZETA", "WN")):
            self.ZETA = params["ZETA"]
            self.WN = params["WN"]
            self.KI = params.get("M", DEFAULTS["M"])
            self.KP = 2 * self.ZETA * self.KI
            self.param_mode = "pole"
        
        self.KBW = params.get("KBW", 0.0)
        self.U_MAX = params.get("U_MAX", DEFAULTS["U_MAX"])
        self.U_MIN = params.get("U_MIN", DEFAULTS["U_MIN"])
        self.DT = params.get("DT", DEFAULTS["DT"])
        self.RX = params.get("RX", DEFAULTS["RX"])
        self.FF = params.get("FF", 0.0)


    def reset(self):
        self.last_error = 0.0
        self.last_u = 0.0

    def control_matlab(self, ref: float) -> float:
        v = float(self.car.getVel()[0])
        error = ref - v

        delta_u = self.KP * (error - self.last_error) + self.KI * self.DT * error
        u_unsat = self.last_u + delta_u

        u_sat = float(np.clip(u_unsat, self.U_MIN, self.U_MAX))

        # back-calculation anti-windup (applied as a correction to internal integrator)
        u_corr = u_unsat + self.KBW * (u_sat - u_unsat)

        # update states using applied (saturated) value
        self.last_error = error
        self.last_u = u_corr

        return u_sat

    def control_tustin(self,ref: float) -> float:
        # error
        error = ref - self.car.getVel()[0]

        # --- Discrete PI in velocity form (Tustin) ---
        u_unsat = self.last_u + (self.KP + self.KI * self.DT / 2.0) * error + (-self.KP + self.KI * self.DT / 2.0) * self.last_error

        # Feedforward compensation for rolling resistance ---
        # u_unsat += R_X * KFF

        # -Saturation
        u_sat = max(self.U_MIN, min(self.U_MAX, u_unsat))

        # Anti-windup 
        u_aw = u_unsat + self.KBW * (u_sat - u_unsat)

        # update states
        self.last_u = u_aw
        self.last_error = error

        return u_sat 
    

    def control(self,ref:float,mode="matlab"):
        if mode == "matlab":
            return self.control_matlab(ref)
        elif mode == "tustin":
            return self.control_tustin(ref)