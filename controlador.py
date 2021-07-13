from control.matlab import *
from control.timeresp import *
import matplotlib.image
import matplotlib.pyplot as plt
import numpy as np

################################################################################################################
#
#               DIGITAL CONTROL - EE/UFSCAR
#
#   Authors: Leonardo Patrocínio, Leonardo Rezende Novaes, Gustavo Kochi, Julio Petrin, Guilherme Valério, Alexandre Saccioto
#   Bruno Paiva, Wellington Correa
#   Version: 1
#   Last-Update: 06.05.2021
#
#   Info: On hover only, Momentum theory, check motor-blade relationship:
#
#   These codes are used in DIGITAL CONTROL classes. You may use and study by them, however use with caution!
#
################################################################################################################



s = tf('s')

#Parametros do Sistema
m = 0.5
g = 9.79 
I = 223e-3
Ea= 18

#Parametro da helice
Ct = 0.1095
Jm_h = (0.025**2 + 0.1778**2)*(0.013/12) #Inércia da hélice / (espessura^2 + diametro^2) * (massa/12)

#Parametros do Motor
Kt = 2.52e-3 #Constante motor Torque constant
Kv = 3790 #RPM 
Ke = 30/(np.pi*Kv) #Const eletrica
Ra = 0.304 #Resistência do motor
J_m = 0.349e-7 + Jm_h #Rotor inertia
Rrad = 0.1778 #raio hélice

#Paremtro lamina hélice
A = np.pi*(Rrad**2)

#Parametros de ambiente para as diferentes cidades.
humidity = [0, 0.6, 0, 0.6, 0, 0.6, 0, 0, 0, 0]
_rho = [1.1120437980847555, 1.1102661030988181, 1.163745153844459, 1.1629001669399746, 1.1276418102211367, 1.1261374248325662, 1.1692984574920768, 1.1373918452461054, 1.1674829094018055, 1.2245418094124811]
_vset = [1.5872794412630247, 1.588549663831097, 1.5516503071229912, 1.5522139350857533, 1.5763356583216763, 1.5773882052070487, 1.5480577456703342, 1.5696501867894523, 1.5491459855576302, 1.5126019384870883]
_omega = [6015.789082386863, 6020.603225919857, 5880.754663996137, 5882.8908139750065, 5974.312145039153, 5978.301297734715, 5867.138856090567, 5948.974207932025, 5871.263285263419, 5732.761346866066]
local = ['Franca - SP', 'Franca - SP', 'Ribeirão Preto - SP', 'Ribeirão Preto - SP', 'São Carlos - SP', 'São Carlos - SP', 'Tietê - SP', 'Santo André - SP', 'Terra Roxa - SP', 'Vitória - ES']


for i in range(10):
    
    ###########################################################################
    ############################ FT DO SISTEMA ################################
    ###########################################################################
    
    omega_rad = (_omega[i] * np.pi)/30

    B = (Ea*I)/((omega_rad)*(m**2)) #coef viscoso
    V_set = _omega[i] / Kv

    #Função de Transferencia do Motor
    num = (Kt*Rrad)/(Ra*J_m)
    den = [1, ((Ke*Kt)/(Ra*J_m) + (B/ J_m))]
    ft_motor = tf(num,den)

    #Função auxiliar (leva como parametros ambiente, helice e v_set)
    ft_auxiliar = tf((2* Ct * _rho[i] * A * _vset[i]), [m, 0, 0])

    #Função de transferencia do peso
    ft_peso = tf(g,[1,0,0]);

    #Função de transferencia linear e não linear
    ft_sistema_linear = (ft_auxiliar * (ft_motor * Rrad)**2)
    ft_sistema_n_linear = (ft_auxiliar * (ft_motor * Rrad)**2) - ft_peso 

    print("### Função de Transferência do sistema linear ###")
    print(ft_sistema_linear)
    print('\n')

    print("### Função de Transferencia do sistema não linear ###")
    print(ft_sistema_n_linear)
    print('\n')

    ###########################################################################
    ############################## CONTROLADOR ################################
    ###########################################################################

    c_linear = ( 2.7314*(10**5) * (s+0.2621)**2 ) / s
    c_n_linear = ( -0.16767 * (s+0.1745)**2 ) / s

    sys_controlado_linear = (ft_sistema_linear * c_linear)/((ft_sistema_linear * c_linear) + 1)

    x, y = step_response(sys_controlado_linear, T=10, T_num=5000)

    
    plt.plot(x,y)
    plt.xlabel("Tempo (s)")
    plt.ylabel("Amplitude (m)")
    plt.title(f"CONTROLE DE POSIÇÃO: {local[i]}, Umidade: {humidity[i]*100}%")
    
    plt.savefig(f'outputs/{local[i]}_umidade{humidity[i]*100}_continuo.png')

    plt.clf()

    sys_controlado_linear_dis = c2d(sys_controlado_linear, method='tustin', Ts=0.06)

    x2, y2 = step_response(sys_controlado_linear_dis, T=10)
    
    plt.step(x2,y2)
    
    plt.xlabel("Tempo (s)")
    plt.ylabel("Amplitude (m)")
    plt.title(f"CONTROLE DE POSIÇÃO: {local[i]}, Umidade: {humidity[i]*100}%")
    
    plt.savefig(f'outputs/{local[i]}_umidade{humidity[i]*100}_discreto.png')
    plt.clf()