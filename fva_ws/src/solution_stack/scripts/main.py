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
from Controller import Controller, stanley_controller
import math
from Vision import *
from ADAS import *

# ============================
# FLAGS DE CONFIGURAÇÃO
# ============================
ENABLE_CRASH_SUPERVISOR = False # ativa/desativa crash detector por sonar
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

KP = 2.3
KI = 1.2
KD = 0



TS = 0.05
U_MAX, U_MIN = 1.0, 0.0

# === Gráficos interativos ===
matplotlib.use("TkAgg")
plt.ion()
plt.rcParams['figure.figsize'] = (8, 6)



def build_controller(car):
    return Controller(car,KP=KP,KI=KI,KD=KD)



# ---- Vision Function
def vision_func(car):
    params = [[20, 100, 100], [40, 255, 255]]
    # pega image
    frame = np.array(car.getImage(gray=True),copy=True)

    contours = findContours(frame,params)

    if contours:

        largest = max(contours, key=cv2.contourArea)

        M  = cv2.moments(largest)

        if M["m00"] > 0:

            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            h, w = frame.shape[:2]

            cv2.circle(frame, (int(cx), int(cy)), 6, (255, 0, 0), -1)

            cx = cx - w/2
            cy = cy - h/2

            cv2.drawContours(frame, [largest], -1, (255, 255, 0), 2)
        else:
            cx = 0

    return cx,frame

# ============================
# LOOP PRINCIPAL
# ============================
def run(sim_params, v_ref=1.0):
    os.makedirs(LOG_DIR, exist_ok=True)

    car = cp.Car(sim_params)
    car.startMission()

    t_hist, v_hist, t_u_hist, u_hist = [], [], [], [0]

    ctrl = build_controller(car)
    tag = "Matlab"
    print(f"[INFO] Controlador: {tag} | Crash: {'ON' if ENABLE_CRASH_SUPERVISOR else 'OFF'}")

    try:
        car.setVel(0.0)
        init_pos = car.getPos()

        while car.t <= sim_params["ts"]:
            car.step()

            # Use the camera to get the lateral error
            lateral_error,image = vision_func(car)

            # Runs the longitudinal controller
            if ENABLE_CRASH_SUPERVISOR and crash_supervisor(car):
                u = 0.0
            else:
                u = ctrl.control(v_ref)
                u_steer = stanley_controller(car,lateral_error)

            # Sets the input to the car
            car.setU(u)
            car.setSteer(u_steer)

            ########################################
            # plota	
            plt.subplot(311)
            plt.cla()
            plt.gca().imshow(image, origin='lower')
            plt.title('t = %.1f' % car.t)
            
            plt.subplot(312)
            plt.cla()
            t = [traj['t'] for traj in car.traj]
            v_hist = [traj['v'] for traj in car.traj]
            u_hist.append(u)
            plt.plot(t,v_hist)
            plt.ylabel('v[m/s]')
            plt.xlabel('t[s]')
            plt.subplot(313)
            plt.plot(t,u_hist)
            plt.ylabel('a[m/s^2]')
            plt.xlabel('t[s]')
            
            plt.show()
            plt.savefig('Step.png')
            plt.pause(0.01)

    finally:
        car.stopMission()
        if sim_params["save"]: car.save(sim_params["logfile"])

    # salva CSV
    if SAVE_LOG and len(t_hist) > 0 and len(u_hist) > 0:
        Nv = min(len(u_hist), max(len(v_hist) - 1, 0))
        with open(f"{LOG_DIR}log_{tag}.csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(["t","v","u"])
            w.writerows(np.column_stack((v_hist[1:1+Nv], u_hist[:Nv])))
        print(f"[INFO] Log salvo em {LOG_DIR}log_{tag}.csv")
    plt.savefig(f"{LOG_DIR}step_{tag}.png", dpi=150)

# ============================
# MAIN
# ============================
if __name__ == "__main__":
    run(parameters, v_ref=1.0)
