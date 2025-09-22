# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2024/1
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import sys
sys.path.append("/home/fva_ws/src/fundamentos_veiculos_autonomos/simulador")

<<<<<<< HEAD:fva_ws/src/solution_stack/scripts/main.py
import class_car as cp
=======
import class_car as cp # type: ignore
>>>>>>> 5f293008c965d61713ca9ab62861bb5eb1846051:src/fundamentos_veiculos_autonomos/simulador/main.py
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib

import math

import csv

# Enables interactive mode
matplotlib.use("TkAgg")
plt.ion()

plt.rcParams['figure.figsize'] = (8,6)

# Globais
parameters = {	'car_id'	: 0,
				'ts'		: 25.0, 			# tempo da simulacao
				'save'		: True,
				'logfile'	: 'logs/',
			}
	
########################################
# thread de controle de velocidade
########################################

# --- controller parameters ---
KP = 7      # proportional gain
KI = 0.3    # integral gain
KBW = 0.2     # back-calculation gain
K = 1       # plant gain (throttle->velocity)
TAU = 2.6e-8     # plant time constant [s]

U_MAX = 10.0   # max throttle
DT = 0.05     # controller timestep [s]

# predictive limiter
PRED_HORIZON = 10   # prediction horizon [s]
DELTA = 0.05         # overshoot margin [m/s]

M = 6.3

R_X = 2.3478
DEACCELERATION = R_X/M

i_error = 0
CarOff = False

def PID(car, ref):
	"""
	PI controller with feedforward, anti-windup, and predictive limiter
	for throttle-only car model.
	"""
	global i_error

	v = car.getVel()[0]  # current velocity
	error = ref - v
	i_error += error
	# feedforward term
	u_ff = R_X/M

	# PI unsaturated 
	u_pi = KP * error + KI * i_error

	# candidate command
	u_cand = u_ff + u_pi
	u_sat = max(0.0, min(u_cand, U_MAX))  # actuator limits

	# predictive limiter
	predicted_velocity = v + PRED_HORIZON*(u_sat - R_X)
	print(f"Cadidate to accel {u_cand} Predicted Velocity {predicted_velocity}")

	if(predicted_velocity >= (ref + DELTA) * error):
		u_allow = (u_sat - v * K)
		u_allow = max(0.0, min(u_allow, U_MAX))
	else:
		u_allow = U_MAX

	print(f"Allowed accel {u_allow}")

	u = min(u_sat, u_allow)  # final command

	# # anti-windup (back-calculation)
	if u < 0.01:
		i_error = 0

	print(f"Integral error {i_error}")

	return u

def Costdown(car):
	global CarOff
	if not CarOff:
		if car.t < 5.0 or (car.t > 5 and abs(car.getVel()[0] - 1) > 0.1 ):
			v_ref = 1
			u = PID(car,v_ref)
		else:
			u = 0.0
			CarOff = True
	else:
		u = 0.0
	return u 

# Calculates time that the car will take to stop
def TimeToStop(V):
	return V/DEACCELERATION

def DistanceToStop(V):
	time_to_stop = TimeToStop(V)
	return DEACCELERATION*time_to_stop^2/2

#Function to check if the car will crash
#Compares the distance read from the sensor with the predicted stopping distance
def CrashSupervisor(car):
	sonar_reading = car.getDistance()
	v_x = car.getVel()[0]
	stopping_distance = DistanceToStop(v_x)

	if stopping_distance < sonar_reading:
		return True

	return False


def control_func(car):

	# seta direcao
	# car.setSteer(np.deg2rad(2.0*np.sin(car.t)))

	u = PID(car,1)

	car.setU(u)

	# if car.t < 2.0:
	# 	u = 0.0
	# else:
	# 	u = 0.5

	# if car.getVel()[0] < 3:
	# 	car.setVel(4)
	# else:
	# 	car.setVel(5)

	print('Vel: ', car.getVel())
	return u
		
########################################
# thread de visão
########################################
def vision_func(car):
		
	# pega imagem
	image = car.getImage()
	
	# ultrasom
	dist = car.getDistance()
	print('Ultrasonic distance: ', np.round(dist,2))
	
	return image
				
########################################
# executa controle
########################################
def run(parameters):
	
	# plt.figure(1)
	# plt.ion()
	
	# cria comunicação com o carrinho
	car = cp.Car(parameters)
	
	# começa a simulação
	car.startMission()

	# main loop
	accel = [0]
	car.setVel(0.0)
	while car.t <= parameters['ts']:
		
		# lê senores
		car.step()
		
		# funcao de controle
		u = control_func(car)
		
		# funcao de visao
		image = vision_func(car)
		
		########################################
		# plota	
		plt.subplot(311)
		plt.cla()
		plt.gca().imshow(image, origin='lower')
		plt.title('t = %.1f' % car.t)
		
		plt.subplot(312)
		plt.cla()
		t = [traj['t'] for traj in car.traj]
		v = [traj['v'] for traj in car.traj]
		accel.append(u)
		plt.plot(t,v)
		plt.ylabel('v[m/s]')
		plt.xlabel('t[s]')
		plt.subplot(313)
		plt.plot(t,accel)
		plt.ylabel('a[m/s^2]')
		plt.xlabel('t[s]')
		
		plt.show()
		plt.savefig('Step.png')
		plt.pause(0.01)

	# termina a missao
	car.stopMission()
	# salva
	if parameters['save']:
		car.save(parameters['logfile'])

	with open('CoastdownLog.csv', 'w', newline='') as csvfile:
		writer = csv.writer(csvfile)
		writer.writerow(["t","v","u"])
		data_rows = np.column_stack((t,v,accel))
		writer.writerows(data_rows)
	# plt.ioff()
	print('Terminou...')

########################################
########################################
if __name__=="__main__":
	run(parameters)
