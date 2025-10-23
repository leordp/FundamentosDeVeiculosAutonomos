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
import class_ultrasonic as cu

import numpy as np
import cv2
import matplotlib.pyplot as plt
import threading
import time
from Controller import Controller


# cria carrinho
car = cp. Car()
car.startMission()

ultrassonic = cu.Ultrasonic()

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
	"KP": 5,
	"KI": 3,
	"KBW": 0,
}

R_X = 0.6616134
DEACCELERATION = max(R_X / M, 1e-6)
ENABLE_CRASH_SUPERVISOR = True # ativa/desativa crash detector por sonar
SONAR_LIMIT = 4.0              # limite do sonar em metros
SAFETY_FACTOR = 1.2            # >=1.0 -> mais conservador
M = 5.2

control_mode = "matlab" #Yan = "tustin" #Leo

control = Controller(car,PARAM_GRUPO)


def time_to_stop(v):
    return v/DEACCELERATION

def distance_to_stop(v):
    time_to_stop = time_to_stop(v)
    return v * time_to_stop - DEACCELERATION*(time_to_stop ** 2) / 2.0

def crash_supervisor():
	global car, ultrassonic
	sonar = float(ultrassonic.getDistance())
	vx = float(car.getVel()[0])
	s_stop = SAFETY_FACTOR * distance_to_stop(vx)
	trigger = (s_stop > sonar) and (sonar < SONAR_LIMIT)
	if trigger:
		print(f"[CRASH] t={car.t:.2f}s | sonar={sonar:.2f} m | v={vx:.2f} m/s -> freio!")
	return trigger


safety_triger = False

def control_func():
	
	global car
	global refste
	global refvel
	global u_traj
	global control

	time.sleep(2.0)
	
	print("Car connected, initializing controller")

	while not terminar:
		# lê sensores
		car.step()
		
		#Crash ADAS
		if (crash_supervisor()):
			car.setU(0.0)
			terminar = True
			break
		

		# seta direcao
		# car.setSteer(refste)
		
		# u = PID(car,MAIN_VEL)
		# atua
		# if 0.0 
		#car.setVel(refvel)

		# if safety_triger:
		# 	u = 0
		# elif 0.0 < car.t < 15.0:
		# 	u = control.control(1.0,mode=control_mode)
		# else:
		# 	u = 0.0

		print(ultrassonic.getDistance()) 


		
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
