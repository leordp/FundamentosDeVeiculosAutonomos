# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2024/1
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import sys, asyncio, csv, os
import numpy as np
import cv2
import matplotlib.pyplot as plt
from evdev import InputDevice, ecodes, list_devices

import class_car as cp

plt.rcParams['figure.figsize'] = (8,6)

# parâmetros da simulação
parameters = {
    'car_id': 0,
    'ts': 600.0,             # tempo total da simulação
    'save': True,
    'logfile': 'logs/',
}

# limites do carro
STEERMAX = cp.CAR['STEERMAX']
U_MIN, U_MAX = 0.0, 1.0

# Deadzones
DEADZONE_STEER = 0.03
DEADZONE_THROTTLE = 0.02

# variáveis globais controladas pelo joystick
throttle = 0.0
steer_rad = 0.0


def pick_joystick():
    devices = [InputDevice(fn) for fn in list_devices()]
    for d in devices:
        name = (d.name or "").lower()
        if any(k in name for k in ["controller", "joystick", "wireless", "sony", "dualshock", "dualsense"]):
            return d
    raise RuntimeError("Nenhum joystick encontrado.")


def get_absinfo_safe(dev, code, fallback):
    try:
        ai = dev.absinfo(code)
        return (ai.min, ai.max, ai.flat or 0)
    except Exception:
        return fallback


def norm_axis_symmetric(val, amin, amax, flat, deadzone_frac=0.03):
    center = 0.5 * (amax + amin)
    span = max(center - amin, amax - center, 1)
    x = (val - center) / span
    dz = max(deadzone_frac, (flat / span) if span > 0 else 0.0)
    if abs(x) < dz:
        x = 0.0
    return max(-1.0, min(1.0, x))


def norm_axis_unipolar(val, amin, amax, flat, deadzone_frac=0.02):
    span = max(amax - amin, 1)
    x = (val - amin) / span
    x = max(0.0, min(1.0, x))
    if x < deadzone_frac:
        x = 0.0
    return x


async def reader(js, ax_info, rz_info):
    """Lê eventos do joystick e atualiza throttle e steer."""
    global throttle, steer_rad
    ax_min, ax_max, ax_flat = ax_info
    rz_min, rz_max, rz_flat = rz_info

    async for event in js.async_read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_X:  # analógico esquerdo
                x = norm_axis_symmetric(event.value, ax_min, ax_max, ax_flat,
                                        deadzone_frac=DEADZONE_STEER)
                steer_rad = x * STEERMAX

            elif event.code == ecodes.ABS_RZ:  # R2
                throttle = norm_axis_unipolar(event.value, rz_min, rz_max, rz_flat,
                                              deadzone_frac=DEADZONE_THROTTLE)

        elif event.type == ecodes.EV_KEY:
            if event.code == ecodes.BTN_SOUTH and event.value == 1:
                print("Botão X pressionado -> Encerrando...")
                break


########################################
# controle via joystick
########################################
def control_func(car):
    car.setU(throttle)
    car.setSteer(steer_rad)
    print(f"[t={car.t:.2f}] throttle={throttle:.2f}, steer={steer_rad:.3f} rad ({np.rad2deg(steer_rad):.1f}°)")


########################################
# visão
########################################
def vision_func(car):
    image = car.getImage()
    dist = car.getDistance()
    print('Ultrasonic distance: ', np.round(dist,2))
    return image


########################################
# executa a simulação
########################################
def run(parameters):
    global throttle, steer_rad

    plt.figure(1)
    plt.ion()

    # joystick
    js = pick_joystick()
    print(f"Usando joystick: {js.path} ({js.name})")

    ax_info = get_absinfo_safe(js, ecodes.ABS_X, (-32768, 32767, 0))
    rz_info = get_absinfo_safe(js, ecodes.ABS_RZ, (0, 255, 0))
    print(f"[Calibração] ABS_X: {ax_info}")
    print(f"[Calibração] ABS_RZ: {rz_info}")

    loop = asyncio.get_event_loop()
    loop.create_task(reader(js, ax_info, rz_info))  # roda leitor em paralelo

    # cria comunicação com o carrinho
    car = cp.Car(parameters)
    car.startMission()

    # prepara arquivo CSV
    os.makedirs(parameters['logfile'], exist_ok=True)
    csv_path = os.path.join(parameters['logfile'], "log_car.csv")
    csvfile = open(csv_path, 'w', newline='')
    writer = csv.writer(csvfile)
    writer.writerow(["t", "v", "u", "steer", "qx", "qy", "qz", "qw"])

    try:
        while car.t <= parameters['ts']:
            car.step()
            control_func(car)
            image = vision_func(car)

            # pega dados para log
            v = car.v
            u = car.u
            steer_val = steer_rad
            qx, qy, qz, qw = car.getQuaternion()

            writer.writerow([car.t, v, u, steer_val, qx, qy, qz, qw])

            # plotagem
            plt.subplot(211)
            plt.cla()
            plt.gca().imshow(image, origin='lower')
            plt.title('t = %.1f' % car.t)

            plt.subplot(212)
            plt.cla()
            t = [traj['t'] for traj in car.traj]
            v_hist = [traj['v'] for traj in car.traj]
            plt.plot(t, v_hist)
            plt.ylabel('v[m/s]')
            plt.xlabel('t[s]')

            plt.show()
            plt.pause(0.01)

            # deixa o asyncio processar eventos do joystick
            loop.run_until_complete(asyncio.sleep(0))

    except KeyboardInterrupt:
        print("Encerrando por teclado...")

    finally:
        car.stopMission()
        csvfile.close()
        if parameters['save']:
            car.save(parameters['logfile'])
        plt.ioff()
        print(f'Log salvo em {csv_path}')
        print('Terminou...')


########################################
if __name__ == "__main__":
    run(parameters)
