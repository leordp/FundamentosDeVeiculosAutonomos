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
				'ts'		: 30.0, 			# tempo da simulacao
				'save'		: True,
				'logfile'	: 'logs/',
			}
	
########################################
# thread de controle de velocidade
########################################

# --- controller parameters ---
KP = 2.31998  # proportional gain
KI = 1.12637  # integral gain
KBW = KP/KI     # back-calculation gain
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

	# u = PID(car,1)

	# car.setU(u)

	if car.t < 2.0:
		u = 0.0
	else:
		u = PID(car,1)
	car.setU(u)

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

	with open('Log.csv', 'w', newline='') as csvfile:
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
