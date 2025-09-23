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

import class_car as cp # type: ignore
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
KP = 5      # proportional gain
KI = 8    # integral gain
KBW = KP/KI     # AntiWindup gain
KFF = 0.01       # FeedForward Gain

TS = 0.05

U_MAX = 1.0   # max throttle
DT = 0.05     # controller timestep [s]

M = 6.3

R_X = 0.57
DEACCELERATION = R_X/M

prev_error = 0
prev_control = 0
CarOff = False

def PID(car, ref):

	global prev_control,prev_error

	## Descrete PI control implementatio

	error = ref - car.getVel()[0]

	u = prev_control + (KP + KI*TS/2)*error + (-KP+KI*TS/2)*prev_error

	# Feedforward

	u += R_X*KFF

	# Anti Windup

	u_saturated = max(0.0,min(U_MAX,u))
	u += KBW*(u_saturated-u)

	prev_control = u
	prev_error = error

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
	return V*time_to_stop - DEACCELERATION*time_to_stop**2/2

#Function to check if the car will crash
#Compares the distance read from the sensor with the predicted stopping distance
def CrashSupervisor(car):
	sonar_reading = car.getDistance()
	v_x = car.getVel()[0]
	stopping_distance = DistanceToStop(v_x)

	print(f"Stopping Distance {stopping_distance} Vs Sonar Reading {sonar_reading}")

	if stopping_distance > sonar_reading and sonar_reading < 4.0:
		return True

	return False

Crash = False
def control_func(car):

	global Crash
	# seta direcao
	# car.setSteer(np.deg2rad(2.0*np.sin(car.t)))

	if Crash:
		u = 0
	else:
		u = PID(car,1)

	if CrashSupervisor(car):
		print("CRASH!!!!!!")
		u = 0
		Crash = True

	# u = Costdown(car)

	car.setU(u)

	# if car.t < 2.0:
	# 	u = 0.0
	# else:
	# 	u = 0.5

	# if car.getVel()[0] < 3:
	# 	car.setVel(4)
	# else:
	# 	car.setVel(5)
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

	with open('CrashWarningTest.csv', 'w', newline='') as csvfile:
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
