% Parâmetros do Sistema
m = 0.5
g = 9.81
rho1 = 1.1120437980847555
rho2 = 1.1102661030988181
rho3 = 1.163745153844459
rho4 = 1.1629001669399746
rho5 = 1.1276418102211367
rho6 = 1.1261374248325662
rho7 = 1.1692984574920768
rho8 = 1.1373918452461054
rho9 = 1.1674829094018055
rho10 = 1.2245418094124811

I = 223e-3 % Corrente de armadura
%V_set = 1.57667; 
g= 9.79 % Calculado no python
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ea=18 %tensão de armadura
omega_andre_rpm = 6015.79 % Franca
omega_andre = omega_andre_rpm*pi/30 
%omega_andre= 625.811 % rad/s (já convertido)

%Parâmetros da hélice
Ct = 0.1095
Jm_h = (0.025^2 + 0.1778^2)*(0.013/12) %Inércia da hélice / (espessura^2 + diametro^2) * (massa/12)

% Parâmetros do Motor
Kt = 2.52e-3 % Constante motor Torque constant
Kv = 3790 %RPM 
Ke = 30/(pi*Kv) % Const eletrica
Ra = 0.304% Resistência do motor
J_m = 0.349e-7 + Jm_h %Rotor inertia
B = (Ea*I)/((omega_andre)*(m^2)) % coef viscoso
Rrad = 0.1778 % raio hélice
V_set = omega_andre_rpm / Kv

%Hélice
rad_blade = Rrad
A = pi*(rad_blade^2)

%V_set = sqrt((g*m)/(8*rho*A)) %teste
%FT do Motor
num = (Kt * Rrad) / (Ra * J_m) ;
den = [1, ((Ke * Kt) /(Ra * J_m) + (B / J_m))];
FT_motor = tf(num, den);

%¨Teste
FT_auxiliar1 = tf((2 * Ct * rho1 * A * V_set), [m,0,0]);
FT_auxiliar2 = tf((2 * Ct * rho2 * A * V_set), [m,0,0]);
FT_auxiliar3 = tf((2 * Ct * rho3 * A * V_set), [m,0,0]);
FT_auxiliar4 = tf((2 * Ct * rho4 * A * V_set), [m,0,0]);
FT_auxiliar5 = tf((2 * Ct * rho5 * A * V_set), [m,0,0]);
FT_auxiliar6 = tf((2 * Ct * rho6 * A * V_set), [m,0,0]);
FT_auxiliar7 = tf((2 * Ct * rho7 * A * V_set), [m,0,0]);
FT_auxiliar8 = tf((2 * Ct * rho8 * A * V_set), [m,0,0]);
FT_auxiliar9 = tf((2 * Ct * rho9 * A * V_set), [m,0,0]);
FT_auxiliar10 = tf((2 * Ct * rho10 * A * V_set), [m,0,0]);
FT_peso = tf(g,[1,0,0]);
%FT_motor = (Ct * rho * A * (FT_motor * Rrad * V_set)^2
 
%FT_auxiliar = tf([4 * V_set], [m, 0, 0])
z_s1 = (FT_auxiliar1 *(FT_motor * Rrad)^2);
z_sl1 = FT_auxiliar1 *(FT_motor * Rrad)^2 - FT_peso; % Não linearizada
z_s2 = (FT_auxiliar2 *(FT_motor * Rrad)^2);
z_sl2 = FT_auxiliar2 *(FT_motor * Rrad)^2 - FT_peso; % Não linearizada
z_s3 = (FT_auxiliar3 *(FT_motor * Rrad)^2);
z_sl3 = FT_auxiliar3 *(FT_motor * Rrad)^2 - FT_peso ;% Não linearizada
z_s4 = (FT_auxiliar4 *(FT_motor * Rrad)^2);
z_sl4 = FT_auxiliar4 *(FT_motor * Rrad)^2 - FT_peso; % Não linearizada
z_s5 = (FT_auxiliar5 *(FT_motor * Rrad)^2);
z_sl5 = FT_auxiliar5 *(FT_motor * Rrad)^2 - FT_peso; % Não linearizada
z_s6 = (FT_auxiliar6 *(FT_motor * Rrad)^2);
z_sl6 = FT_auxiliar6 *(FT_motor * Rrad)^2 - FT_peso ;% Não linearizada
z_s7 = (FT_auxiliar7 *(FT_motor * Rrad)^2);
z_sl7 = FT_auxiliar7 *(FT_motor * Rrad)^2 - FT_peso; % Não linearizada
z_s8 = (FT_auxiliar8 *(FT_motor * Rrad)^2);
z_sl8 = FT_auxiliar8 *(FT_motor * Rrad)^2 - FT_peso; % Não linearizada
z_s9 = (FT_auxiliar9 *(FT_motor * Rrad)^2);
z_sl9 = FT_auxiliar9 *(FT_motor * Rrad)^2 - FT_peso; % Não linearizada
z_s10 = (FT_auxiliar10 *(FT_motor * Rrad)^2);
z_sl10 = FT_auxiliar10 *(FT_motor * Rrad)^2 - FT_peso; % Não linearizada

MF1 = (CL * z_s1) / ( 1 +(CL * z_s1));
MF1_NL = (CNL * z_sl1) / ( 1 +(CNL * z_sl1));
MF2 = (CL * z_s2) / ( 1 +(CL * z_s2));
MF2_NL = (CNL * z_sl2) / ( 1 +(CNL * z_sl2));
MF3 = (CL * z_s3) / ( 1 +(CL * z_s3));
MF3_NL = (CNL * z_sl3) / ( 1 +(CNL * z_sl3));
MF4 = (CL * z_s4) / ( 1 +(CL * z_s4));
MF4_NL = (CNL * z_sl4) / ( 1 +(CNL * z_sl4));
MF5 = (CL * z_s5) / ( 1 +(CL * z_s5));
MF5_NL = (CNL * z_sl5) / ( 1 +(CNL * z_sl5));
MF6 = (CL * z_s6) / ( 1 +(CL * z_s6));
MF6_NL = (CNL * z_sl6) / ( 1 +(CNL * z_sl6));
MF7 = (CL * z_s7) / ( 1 +(CL * z_s7));
MF7_NL = (CNL * z_sl7) / ( 1 +(CNL * z_sl7));
MF8 = (CL * z_s8) / ( 1 +(CL * z_s8));
MF8_NL = (CNL * z_sl8) / ( 1 +(CNL * z_sl8));
MF9 = (CL * z_s9) / ( 1 +(CL * z_s9));
MF9_NL = (CNL * z_sl9) / ( 1 +(CNL * z_sl9));
MF10 = (CL * z_s10) / ( 1 +(CL * z_s10));
MF10_NL = (CNL * z_sl10) / ( 1 +(CNL * z_sl10));

CL_Z = c2d(CL, 0.06, 'tustin');
CNL_Z = c2d(CNL, 0.06, 'tustin');

FTL_Z = c2d(z_s1, 0.06, 'matched');
FTNL_Z = c2d(z_sl1, 0.06, 'matched');

MFL_Disc_Data = get (out, 'MFL_Disc');
MFL_Disc_Noise_Data = get (out, 'MFL_Disc_Noise');
MFNL_Disc_Data = get (out, 'MFNL_Disc');
MFNL_Disc_Noise_Data = get (out, 'MFNL_Disc_Noise');
MFL_Data = get(out, 'MFL');
MFNL_Data = get (out, 'MFNL');
PIDL_Data = get (out, 'PIDL');
PIDNL_Data = get (out, 'PIDNL');

figure(1)
plot(MFL_Disc_Data)
hold on
plot(MFL_Disc_Noise_Data)
legend ('Sem Ruído','Com Ruído')
title('Sistema linear discretizado')
ylabel 'Altitude (m)';
xlabel 'Tempo (s)';

figure(2)
plot(MFNL_Disc_Data)
hold on
plot(MFNL_Disc_Noise_Data)
legend ('Sem Ruído','Com Ruído')
title('Sistema não linear discretizado')
ylabel 'Altitude (m)';
xlabel 'Tempo (s)';

figure(3)
subplot(2,1,1)
plot (MFL_Data)
title ('Resposta ao degrau sistema linear')

subplot(2,1,2)
plot(PIDL_Data)
title ('Ação de controle linear')

figure(4)
subplot(2,1,1)
plot (MFNL_Data)
title ('Resposta ao degrau sistema não-linear')

subplot(2,1,2)
plot(PIDNL_Data)
title ('Ação de controle não-linear')


