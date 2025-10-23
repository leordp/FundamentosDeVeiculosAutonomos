# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075)
# Fundamentos de Veículos Autônomos - 2024/1
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# DELT – Escola de Engenharia - UFMG
########################################

import sys, csv, os
sys.path.append("/home/fva_ws/src/fundamentos_veiculos_autonomos/simulador")

import class_car as cp  # type: ignore
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from Controller import Controller

# ============================
# FLAGS DE CONFIGURAÇÃO
# ============================
CONTROLLER = "v2"              # "v1" (PI incremental) ou "v2" (PI Tustin)
ENABLE_CRASH_SUPERVISOR = False # ativa/desativa crash detector por sonar
SONAR_LIMIT = 4.0              # limite do sonar em metros
SAFETY_FACTOR = 1.2            # >=1.0 -> mais conservador
SAVE_LOG = True
LOG_DIR = "logs/"

# ============================
# PARÂMETROS GERAIS
# ============================
parameters = {
    "car_id": 0,
    "ts": 25.0,      # tempo total da simulação [s]
    "save": True,    # salva .npz do simulador
    "logfile": LOG_DIR,
}

PARAM_LEO = {
	"KBW": 1.5,
	"WN" : 1,
	"ZETA": 1,
	"KFF": 0.05,
}

PARAM_YAN = {
	"KP": 5,
	"KI": 2,
	"KBW": 0
}

PARAM_GRUPO = {
	"KP": 2.3,
	"KI": 1.2,
	"KBW": 0,
}


TS = 0.05
U_MAX, U_MIN = 1.0, 0.0
M = 6.3
R_X = 0.6616134
DEACCELERATION = max(R_X / M, 1e-6)

# === Gráficos interativos ===
matplotlib.use("TkAgg")
plt.ion()
plt.rcParams['figure.figsize'] = (8, 6)

# ============================
# UTILIDADES
# ============================
def time_to_stop(v):
    return v/DEACCELERATION

def distance_to_stop(v):
    t_to_stop = time_to_stop(v)
    return v * t_to_stop - DEACCELERATION*(t_to_stop ** 2) / 2.0

def crash_supervisor(car):
    sonar = float(car.getDistance())
    vx = float(car.getVel()[0])
    s_stop = SAFETY_FACTOR * distance_to_stop(vx)
    trigger = (s_stop > sonar) and (sonar < SONAR_LIMIT)
    if trigger:
        print(f"[CRASH] t={car.t:.2f}s | sonar={sonar:.2f} m | v={vx:.2f} m/s -> freio!")
    return trigger

# ============================
# CONTROLADORES
# ============================
class ControllerV1:
    def __init__(self):
        self.KP, self.KI = 2.31998, 1.12637
        self.KBW = self.KP / max(self.KI, 1e-9)
        self.dt = TS
        self.e_prev, self.u_prev = 0.0, 0.0
    def step(self, v_meas, v_ref):
        e = v_ref - v_meas
        delta_u = self.KP * (e - self.e_prev) + self.KI * self.dt * e
        u_unsat = self.u_prev + delta_u
        u_sat = float(np.clip(u_unsat, U_MIN, U_MAX))
        self.e_prev, self.u_prev = e, u_sat
        return u_sat

class ControllerV2:
    def __init__(self):
        self.KI, self.KP = M, 2.0 * 1.0 * M
        self.KBW, self.dt = 1.5, TS
        self.e_prev, self.u_prev = 0.0, 0.0
    def step(self, v_meas, v_ref):
        e = v_ref - v_meas
        a, b = (self.KP + self.KI * self.dt / 2.0), (-self.KP + self.KI * self.dt / 2.0)
        u_unsat = self.u_prev + a * e + b * self.e_prev
        u_sat = max(U_MIN, min(U_MAX, u_unsat))
        u_aw = u_unsat + self.KBW * (u_sat - u_unsat)
        self.u_prev, self.e_prev = u_aw, e
        return u_sat

def build_controller(car):
    return Controller(car,PARAM_GRUPO)
    # return (ControllerV1(), "v1") if name == "v1" else (ControllerV2(), "v2")

# ============================
# LOOP PRINCIPAL
# ============================
def run(sim_params, controller_name, v_ref=1.0):
    os.makedirs(LOG_DIR, exist_ok=True)

    car = cp.Car(sim_params)
    car.startMission()

    t_hist, v_hist, t_u_hist, u_hist = [], [], [], []

    ctrl = build_controller(car)
    tag = "Matlab"
    print(f"[INFO] Controlador: {tag} | Crash: {'ON' if ENABLE_CRASH_SUPERVISOR else 'OFF'}")

    try:
        car.setVel(0.0)
        while car.t <= sim_params["ts"]:
            car.step()
            if ENABLE_CRASH_SUPERVISOR and crash_supervisor(car):
                u = 0.0
            else:
                u = ctrl.control(v_ref)
            car.setU(u)

            # históricos
            t_hist = [traj["t"] for traj in car.traj]
            v_hist = [traj["v"] for traj in car.traj]
            t_u_hist.append(car.t)
            u_hist.append(u)

            # plots
            plt.subplot(3, 1, 1); plt.cla()
            plt.gca().imshow(car.getImage(), origin="lower")
            plt.title(f"t={car.t:.2f}s")

            plt.subplot(3, 1, 2); plt.cla()
            plt.plot(t_hist, v_hist, label="v [m/s]")
            plt.axhline(v_ref, linestyle="--", label="ref", linewidth=1)
            plt.ylabel("Vel [m/s]"); plt.xlabel("t [s]"); plt.grid(True); plt.legend()

            plt.subplot(3, 1, 3); plt.cla()
            plt.plot(t_u_hist, u_hist, label="u (throttle)")
            plt.ylabel("u [-]"); plt.xlabel("t [s]"); plt.grid(True); plt.legend()

            plt.tight_layout(); plt.pause(0.01)

    finally:
        car.stopMission()
        if sim_params["save"]: car.save(sim_params["logfile"])

    # salva CSV
    if SAVE_LOG and len(t_hist) > 0 and len(u_hist) > 0:
        Nv = min(len(u_hist), max(len(v_hist) - 1, 0))
        with open(f"{LOG_DIR}log_{tag}.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["t","v","u"])
            w.writerows(np.column_stack((t_u_hist[:Nv], v_hist[1:1+Nv], u_hist[:Nv])))
        print(f"[INFO] Log salvo em {LOG_DIR}log_{tag}.csv")
    plt.savefig(f"{LOG_DIR}step_{tag}.png", dpi=150)

# ============================
# MAIN
# ============================
if __name__ == "__main__":
    run(parameters, CONTROLLER, v_ref=1.0)
