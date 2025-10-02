#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

# ------------------ Parâmetros ------------------
npz_path = "dados_kp5_ki3_deucertotalvez.npz"   # caminho do arquivo NPZ

# ------------------ Carrega NPZ ------------------
data = np.load(npz_path, allow_pickle=True)
arr = data['data']

# Extrai cada campo
t   = np.array([d['t']   for d in arr])
v   = np.array([d['v']   for d in arr])
vref= np.array([d['vref']for d in arr])
th  = np.array([d['th']  for d in arr])
w   = np.array([d['w']   for d in arr])
u   = np.array([d['u']   for d in arr])
a   = np.array([d['a']   for d in arr])
px  = np.array([d['p'][0] for d in arr])
py  = np.array([d['p'][1] for d in arr])

# ------------------ Plot 1: Velocidade real x referência ------------------
plt.figure(figsize=(10,5))
plt.plot(t, v, label='Velocidade real (v)')
plt.plot(t, vref, '--', label='Velocidade de referência (vref)')
plt.xlabel('Tempo (s)')
plt.ylabel('Velocidade (m/s)')
plt.title('Velocidade real vs referência')
plt.legend()
plt.grid(True)
plt.tight_layout()

# ------------------ Plot 2: Sinal de controle ------------------
plt.figure(figsize=(10,4))
plt.plot(t, u, color='tab:red')
plt.xlabel('Tempo (s)')
plt.ylabel('u')
plt.title('Sinal de controle')
plt.grid(True)
plt.tight_layout()

# ------------------ Plot 3: Velocidade angular e orientação ------------------
fig, ax1 = plt.subplots(figsize=(10,5))
ax1.plot(t, th, label='Orientação (th)', color='tab:blue')
ax1.set_xlabel('Tempo (s)')
ax1.set_ylabel('th (rad)', color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')

ax2 = ax1.twinx()
ax2.plot(t, w, label='Velocidade angular (w)', color='tab:orange')
ax2.set_ylabel('w (rad/s)', color='tab:orange')
ax2.tick_params(axis='y', labelcolor='tab:orange')

fig.suptitle('Orientação e Velocidade Angular')
fig.tight_layout()
plt.grid(True)

# ------------------ Plot 4: Aceleração ------------------
plt.figure(figsize=(10,4))
plt.plot(t, a, color='tab:green')
plt.xlabel('Tempo (s)')
plt.ylabel('Aceleração (m/s²)')
plt.title('Aceleração')
plt.grid(True)
plt.tight_layout()

# ------------------ Plot 5: Trajetória XY ------------------
plt.figure(figsize=(6,6))
plt.plot(px, py, label='Trajetória')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajetória no plano XY')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.tight_layout()

# ------------------ Mostrar tudo ------------------
plt.show()
