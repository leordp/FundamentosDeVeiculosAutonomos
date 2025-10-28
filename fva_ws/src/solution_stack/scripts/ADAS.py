
M = 6.3
SONAR_LIMIT = 4.0              # limite do sonar em metros
SAFETY_FACTOR = 1.2            # >=1.0 -> mais conservador
R_X = 0.6616134
DEACCELERATION = max(R_X / M, 1e-6)


# ============================
# UTILIDADES
# ============================
def time_to_stop(v):
    return v/DEACCELERATION

def distance_to_stop(v):
    t_to_stop = time_to_stop(v)
    return v * t_to_stop - DEACCELERATION*(t_to_stop ** 2) / 2.0

def crash_supervisor(car,cu=None):
    if cu == None:
        sonar = float(car.getDistance())
    else:
        sonar = float(cu.getDistance)
    vx = float(car.getVel()[0])
    s_stop = SAFETY_FACTOR * distance_to_stop(vx)
    trigger = (s_stop > sonar) and (sonar < SONAR_LIMIT)
    if trigger:
        print(f"[CRASH] t={car.t:.2f}s | sonar={sonar:.2f} m | v={vx:.2f} m/s -> freio!")
    return trigger