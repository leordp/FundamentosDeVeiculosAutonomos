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
from Controller import Controller, stanley_controller
from ADAS import *
from Vision import *


#Image Resolution
WIDTH = 640
HEIGHT = 480

# cria carrinho
car = cp.Car()
car.startMission()

ultrassonic = cu.Ultrasonic()

terminar = False

MAIN_VEL = 1.0

refvel = MAIN_VEL
refste = np.deg2rad(0.0)

lateral_error = 0

########################################
# thread de controle de velocidade

KP = 3
KI = 5
KD = 0

ENABLE_CRASH_SUPERVISOR = False # ativa/desativa crash detector por sonar

control = Controller(car,KP=KP,KD=KD,KI=KI)

def control_func():
	
	global car
	global refvel
	global control
	global lateral_error

	time.sleep(2.0)
	
	print("Car connected, initializing controller")

	while not terminar:
		# lê sensores
		car.step()
		
		u = control.control(refvel)
		u_steer = stanley_controller(car,lateral_error)

		# #Crash ADAS
		# if (crash_supervisor(car,cu) and ENABLE_CRASH_SUPERVISOR):
		# 	car.setU(0.0)
		# 	terminar = True
		# 	break
		
		
		
		car.setU(u)
		car.setSteer(u_steer)
		
		# espera
		time.sleep(0.005)
		


def vision_func():
	
	global car
	global refste
	global refvel
	global frame
	

	while not terminar:
		
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

		global lateral_error
		lateral_error = cx

		
########################################
# disparar threads
thread_control = threading.Thread(target=control_func)
thread_vision = threading.Thread(target=vision_func)
thread_control.start()
thread_vision.start()

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
thread_vision.join()
plt.pause(1.0)


thread_control.join()
thread_vision.join()
del car
print('Terminou...')
