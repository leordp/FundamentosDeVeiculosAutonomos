#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# ------------------ Parâmetros ------------------
npz_path = "controlador_kp5_ki3.npz"
t_start = 15.2
t_end   = 16.5

# ------------------ Carrega NPZ ------------------
data = np.load(npz_path, allow_pickle=True)
arr = data['data']

t = np.array([d['t'] for d in arr])
v = np.array([d['v'] for d in arr])

# ------------------ Seleção do intervalo ------------------
mask = (t >= t_start) & (t <= t_end)
t_seg = t[mask]
v_seg = v[mask]

# ------------------ Função linear para ajuste ------------------
def linear_func(t, a, b):
    return a * t + b

# Ajuste usando curve_fit
popt, pcov = curve_fit(linear_func, t_seg, v_seg)
a_fit, b_fit = popt

# ------------------ R² ------------------
v_pred = linear_func(t_seg, a_fit, b_fit)
ss_res = np.sum((v_seg - v_pred)**2)
ss_tot = np.sum((v_seg - np.mean(v_seg))**2)
r2 = 1 - ss_res/ss_tot

# ------------------ Curva da reta no intervalo ------------------
t_model = np.linspace(t_seg.min(), t_seg.max(), 200)
v_model = linear_func(t_model, a_fit, b_fit)

# ------------------ Plotagem ------------------
plt.figure(figsize=(10,5))
plt.scatter(t, v, s=10, label='Todos os dados (NPZ)', alpha=0.4)
plt.scatter(t_seg, v_seg, s=20, label='Dados usados no ajuste', color='tab:blue')
plt.plot(t_model, v_model, 'r', label=f'Reta ajustada (curve_fit)\n v = {a_fit:.3f} t + {b_fit:.3f}\nR² = {r2:.4f}')
plt.axvline(t_start, color='gray', linestyle='--', alpha=0.7)
plt.axvline(t_end, color='gray', linestyle='--', alpha=0.7)
plt.xlabel('Tempo (s)')
plt.ylabel('Velocidade (m/s)')
plt.title(f'Ajuste Linear com curve_fit (Intervalo {t_start}s – {t_end}s)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# ------------------ Resultados ------------------
print("\n--- RESULTADOS DO AJUSTE ---")
print(f"Inclinação (a): {a_fit:.6f} m/s²")
print(f"Intercepto (b): {b_fit:.6f} m/s")
print(f"R²: {r2:.6f}")
