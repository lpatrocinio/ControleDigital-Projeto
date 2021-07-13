################################################################################################################
#
#               DIGITAL CONTROL - EE/UFSCAR
#
#   Author: André Carmona Hernandes
#   Version: 1
#   Last-Update: 06.05.2021
#
#   Info: On hover only, Momentum theory, check motor-blade relationship:
#
#   These codes are used in DIGITAL CONTROL classes. You may use and study by them, however use with caution!
#
################################################################################################################

from control.matlab import *
import matplotlib.pyplot as plt
import numpy as np

# Equations used:
# In Brazil, it is common to use 0.5*pho*Ct*(pi*R^2)*(omega*R)^2, however,
# https://m-selig.ae.illinois.edu/props/propDB.html uses a different convention, thus, to use the measured values,
# a scaling must be done

## Inputs
# Inputs of the blade
# TODO: Read txt from database and que the constants.
c_tu = 0.0994
c_pu = 0.0424
diameter = 8
# https://m-selig.ae.illinois.edu/props/volume-1/plots/apcsf_11x4.7_static_ct.png
# https://m-selig.ae.illinois.edu/props/volume-1/plots/apcsf_11x4.7_static_cp.png

# drone
mass = 0.5
des_control = 0.4  # 35% for hover, it can be 40, but as there are some approximations, I am keeping at 35.
v_bat = 11.1  # 3S
n_motors = 4

# Motor
# https://br.banggood.com/IFlight-XING-E-Pro-2207-1800KV-3-6S-or-2450KV-2750KV-2-4S-Brushless-Motor-for-RC-Drone-FPV-Racing-p-1518196.html?cur_warehouse=CN&ID=47980&rmmds=search
KV = 1680 # rpm/V
I_max = -1.35  # max current
R_motor = 0.6

# Enviroment
# input
#height = 860  # meters above seal level
#latitude = -22
#humidity = 0

local = ['Franca - SP', 'Franca - SP', 'Ribeirão Preto - SP', 'São Carlos - SP', 'São Carlos - SP', 'Tietê - SP', 'Santo André - SP', 'Terra Roxa - SP', 'Vitória - ES', 'Vitória - ES']
height = [996, 996, 531, 854, 854, 482, 766, 498, 4, 4]
latitude = [-20.5418, -20.5418, -21.1767, -22.0154, -22.0154, -23.1123 ,-23.6666, -20.7895, -20.3222, -20.3222]
humidity = [0, 0.6, 0.6, 0, 0.6, 0, 0.6, 0, 0, 0.6]

for i in range(10):
    print("################# " + local[i] + " #################")
    
    # blade
    radius = diameter * 25.4 / 2000
    ct_br = (8 / (np.pi ** 3)) * c_tu
    cq_br = (8 / (np.pi ** 4)) * c_pu

    # Atmospheric calculation

    # constants
    vapor_density = humidity[i] * 12.83 / 1000  # kg/m^3
    M_dry = 0.0289652
    M_water = 0.018016
    R_gas = 8.31446
    T0 = 288.15
    L = 0.0065
    p0 = 101325

    # equations

    temp = T0 - L * height[i]
    lat_r = np.deg2rad(latitude[i])
    g = 9.780327 * (1 + 0.0053024 * (np.sin(lat_r)) ** 2 - 0.0000058 * (np.sin(2 * lat_r)) ** 2)
    # Buck equation for Vapour pressure
    p_vapour = 0.61121 * np.exp((18.678 - (temp - 273.15) / 234.5) * ((temp - 273.15) / (257.14 + (temp - 273.15))))
    # dry pressure
    p_dry = p0 * (1 - L * height[i] / T0) ** (9.81 * M_dry / (R_gas * L))

    rho_dry = p_dry * M_dry / (R_gas * temp)
    hum_mass = vapor_density / rho_dry
    partial_dry = (1 - hum_mass) * p_dry
    partial_wet = hum_mass * p_vapour * 100000
    air_density = (partial_dry * M_dry + partial_wet * M_water) / (R_gas * temp)

    # motor
    kw = 30 / (np.pi * KV)
    kt = 1.5 * kw / np.sqrt(3)
    if I_max > 0:
        r_est = v_bat / I_max
    else:
        r_est = R_motor
    # T_hover = P/n_motors

    omega = np.sqrt(2 * mass * g / (air_density * n_motors * ct_br * np.pi * (radius ** 4)))
    print(omega * 30 / np.pi)

    # ideal torque
    Q = 0.5 * air_density * cq_br * np.pi * (radius ** 5) * (omega ** 2)
    Aq = r_est * Q / (kt * v_bat)
    # motor BEMF
    Q_bemf = kw * omega
    Bq = Q_bemf / v_bat

    # demanded
    Q_used = Aq + Bq

    # Desired Q

    print(des_control - Q_used)

    current = (Q_used * v_bat - kw * omega) / r_est
    print(current)

