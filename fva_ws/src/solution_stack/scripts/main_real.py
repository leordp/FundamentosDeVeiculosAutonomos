# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import sys
sys.path.append("/home/alunos/Desktop/fva2025/GrupoDoRaviEDoYan/fva_ws/src/fundamentos_veiculos_autonomos/veiculo_real")

import class_car as cp
import numpy as np
import cv2
import matplotlib.pyplot as plt
import threading
import time


# cria carrinho
car = cp. Car()
car.startMission()

terminar = False

MAIN_VEL = 1.0

refvel = MAIN_VEL
refste = np.deg2rad(0.0)
#frame = car.getImage(gray=True)
#W = frame.shape[1]
#H = frame.shape[0]

#plt.ion()
#plt.figure(1)

########################################
# thread de controle de velocidade


# --- controller parameters ---

M = 6.3          # mass [kg]

WN = 1  # Natural Frequency
ZETA = 1 # Damping Ratio

KI = M
KP = 2 * ZETA * KI


KBW = 1.5    # Anti-windup back-calculation gain
KFF = 0.05       # Feedforward gain

TS = 0.05        # controller sample time [s]
U_MAX = 1.0      # throttle upper limit
U_MIN = 0.0      # throttle lower limit


R_X = 0.6616134  # rolling resistance force [N]
DEACCELERATION = R_X / M

# state variables
prev_error = 0.0
prev_control = 0.0

# --- controller parameters ---
KP = 5  # proportional gain
KI = 2  # integral gain
KBW = 0     # back-calculation gain
K = 1       # plant gain (throttle->velocity)
TAU = 2.6e-8     # plant time constant [s]

U_MAX = 1.0   # max throttle
DT = 0.05     # controller timestep [s]

# predictive limiter
PRED_HORIZON = 10   # prediction horizon [s]
DELTA = 0.05         # overshoot margin [m/s]

M = 6.3

R_X = 2.3478
DEACCELERATION = R_X/M

i_error = 0
CarOff = False
erro_anterior = 0
u_anterior = 0

erro_anterior = 0.0
u_anterior    = 0.0
i_aw          = 0.0  # estado de anti-windup (opcional se quiser separar)

def PID(car, ref):
    global erro_anterior, u_anterior, i_aw

    v = car.getVel()[0]
    e = ref - v

    # incremento não saturado
    delta_u = KP * (e - erro_anterior) + KI * DT * e
    u_unsat = u_anterior + delta_u

    # saturação
    U_MIN = 0.0
    u_sat = float(np.clip(u_unsat, U_MIN, U_MAX))

    # retrocálculo (empurra o erro de saturação pro integrador incremental)
    # equivalente a somar KBW*(u_sat - u_unsat) no estado "integral"
    u_corr = u_unsat + KBW * (u_sat - u_unsat)
    # OBS: a saída aplicada é u_sat, não u_corr

    # atualizações
    erro_anterior = e
    u_anterior    = u_sat  # guarde o que foi APLICADO (saturado)

    return u_sat

def PID_LEO(car, ref):
    global prev_control, prev_error

    # error
    error = ref - car.getVel()[0]

    # --- Discrete PI in velocity form (Tustin) ---
    u_unsat = prev_control + (KP + KI * TS / 2.0) * error + (-KP + KI * TS / 2.0) * prev_error

    # Feedforward compensation for rolling resistance ---
    # u_unsat += R_X * KFF

    # -Saturation
    u_sat = max(U_MIN, min(U_MAX, u_unsat))

    # Anti-windup 
    u_aw = u_unsat + KBW * (u_sat - u_unsat)

    # update states
    prev_control = u_aw
    prev_error = error

    return u_sat 



def control_func():
	
	global car
	global refste
	global refvel
	global u_traj
	
	while not terminar:
		# lê sensores
		car.step()
		
		# seta direcao
		# car.setSteer(refste)
		
		# u = PID(car,MAIN_VEL)
		# atua
		# if 0.0 
		#car.setVel(refvel)
		
		if 0.0 < car.t < 60.0:
			u = PID(car, 1.0)
		else:
			u = 0.0
		
	#	if 2.0 < car.t < 5.0:
	#		u = 1.0
	#	else:
	#		u = 0.0
		
		print(car.t)
		car.setU(u)
		
		# espera
		time.sleep(0.005)
		
########################################
# thread de visão
def vision_func():
	
	global car
	global refste
	global refvel
	global frame
	
	# Load the dictionary of ArUco markers.
	aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
	# Create a parameters object for ArUco detection.
	parameters = cv2.aruco.DetectorParameters_create()
	
	while not terminar:
		
		# pega image
		frame = car.getImage(gray=True)
		
		# Detect ArUco markers in the frame.
		corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
		
		# velocidade padrão
		refvel = MAIN_VEL
		
		# nao vi nada, continua
		if ids is None:
			continue
		
		# Iterate through detected markers
		for i, marker_id in enumerate(ids):
			
			# se vir o aruco 24
			if np.squeeze(marker_id) == 24:
							
				# Get the corner coordinates of the current marker
				marker_corners = corners[i][0]
				
				# Calculate the centroid of the marker
				centroid_x = int(sum(marker_corners[:, 0]) / 4)
				centroid_y = int(sum(marker_corners[:, 1]) / 4)
				
				# aumenta velocidade de referencia
				refvel = 1.5*MAIN_VEL
				
				# estercamento aponta para o aruco
				cx = centroid_x - W/2
				refste = -np.deg2rad(20.0*cx/(W/2))

########################################
# disparar threads
thread_control = threading.Thread(target=control_func)
#thread_vision = threading.Thread(target=vision_func)
thread_control.start()
#thread_vision.start()

########################################
# loop principal
while car.t < 75.0:
	
	# plota
	plt.subplot(211)
	plt.cla()
	#plt.gca().imshow(frame, cmap='gray')
		
	plt.subplot(212)
	plt.cla()
	t = [traj['t'] for traj in car.traj]
	v = [traj['v'] for traj in car.traj]
	u = [traj['u'] for traj in car.traj]
	vref = [MAIN_VEL for traj in car.traj]
	plt.plot(t, v, 'k')
	plt.plot(t, vref, 'r--')
	plt.plot(t,u,'b.')
	plt.ylabel('Vel')
	plt.xlabel('Time')
	
	#plt.show()
	#plt.pause(1.0)

# desliga o carro
car.setU(0.0)
plt.savefig('Graph_2.png')
terminar = True

print("salvando trajetoria")
car.saveTraj()
car.save('logautal')
print("trajetoria salva")



# junta as threads
thread_control.join()
#thread_vision.join()
plt.pause(1.0)


thread_control.join()
#thread_vision.join()
del car
print('Terminou...')
