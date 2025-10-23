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
				'ts'		: 100.0, 			# tempo da simulacao
				'save'		: True,
				'logfile'	: 'logs/',
			}
	
########################################
# thread de controle de velocidade
########################################

# --- controller longitudinal ---
KP = 2.31998  # proportional gain
KI = 1.12637  # integral gain
KBW = KP/KI     # back-calculation gain

U_MIN = 0.0
U_MAX = 1.0

DT = 0.05

erro_anterior = 0.0
u_anterior    = 0.0
i_aw          = 0.0 

KP_STANLEY = 1e-3


erro_lateral = 0.0

def ControladorLongitudinal(car, ref):
    global erro_anterior, u_anterior, i_aw

    v = car.getVel()[0]
    e = ref - v

    # incremento não saturado
    delta_u = KP * (e - erro_anterior) + KI * DT * e
    u_unsat = u_anterior + delta_u

    # saturação
    u_sat = float(np.clip(u_unsat, U_MIN, U_MAX))

    # retrocálculo (empurra o erro de saturação pro integrador incremental)
    # equivalente a somar KBW*(u_sat - u_unsat) no estado "integral"
    u_corr = u_unsat + KBW * (u_sat - u_unsat)
    # OBS: a saída aplicada é u_sat, não u_corr

    # atualizações
    erro_anterior = e
    u_anterior    = u_sat  # guarde o que foi APLICADO (saturado)

    return u_sat

def ControladorStanley(car):
    global erro_lateral, KP_STANLEY

    v = car.getVel()[0]
    print('yaw', car.getYaw())
	
    direcao_atual = np.deg2rad(car.getYaw())


    direcao_desejada = np.arctan2((KP_STANLEY * erro_lateral)/v, 1.0)
	
    direcao_desejada = np.clip(direcao_desejada, np.deg2rad(-20), np.deg2rad(20))

    return direcao_desejada

def CriaTrajetoria(car):
	pass

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

def control_func(car):

	# seta direcao
	# car.setSteer(np.deg2rad(2.0*np.sin(car.t)))



	# u = PID(car,1)

	# car.setU(u)

	if 0.01 > car.t:
		u = 0.0
	else:
		u = ControladorLongitudinal(car,1)
		delta =  ControladorStanley(car)
	car.setU(u)
	car.setSteer(delta)

	# if car.getVel()[0] < 3:
	# 	car.setVel(4)
	# else:
	# 	car.setVel(5)

	print('Vel: ', car.getVel())
	return u
		
########################################
# thread de visão
########################################
def vision_func(car, DEBUG=False):
    # captura imagem e faz cópia mutável
    image = np.array(car.getImage(), copy=True)

    # === CORRIGE A ORIENTAÇÃO ===
    # Se a imagem estiver invertida verticalmente, ative o flip
    image = cv2.flip(image, 0)  # desative se já estiver correta

    # converte RGB → HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    # === CONFIGURAÇÃO DA COR DA LINHA (amarela) ===
    lower = np.array([20, 100, 100])
    upper = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)

    # === TRATAMENTO DA MÁSCARA ===
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # === ENCONTRA CONTORNOS ===
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cx_line, cy_line = None, None
    h, w = mask.shape

    if contours:
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] > 0:
            cx_line = int(M["m10"] / M["m00"])
            cy_line = int(M["m01"] / M["m00"])
            # desenha o contorno e o centro da linha
            cv2.drawContours(image, [largest], -1, (255, 255, 0), 2)
            cv2.circle(image, (cx_line, cy_line), 6, (255, 0, 0), -1)

    # === CENTRO DA CÂMERA ===
    cx_cam = w // 2
    cv2.line(image, (cx_cam, 0), (cx_cam, h), (0, 255, 0), 2)

    # === ERRO LATERAL ===
    if cx_line is not None:
        erro_px = cx_line - cx_cam
        erro_norm = erro_px / (w / 2)
        print(f"Erro lateral: {erro_px:.1f} px  ({erro_norm:.2f} relativo)")
    else:
        erro_px = 0
        erro_norm = 0
        print("⚠️ Linha não detectada")

    # === ULTRASSOM ===
    dist = car.getDistance()
    print('Ultrasonic distance: ', np.round(dist, 2))

    # === DEBUG VISUAL (opcional) ===
    if DEBUG:
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        debug_top = np.hstack((image, mask_color))
        debug = np.vstack((debug_top, np.zeros_like(debug_top)))
        cv2.imshow("Camera / Mask / ROI", cv2.cvtColor(debug, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

    global erro_lateral
    erro_lateral = erro_px

    return cv2.flip(image, 0)



				
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
