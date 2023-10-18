clc, clear, close all

alpha = 18.02;
beta =  1.74;
gamma = 0.5;

Gmotor = tf(1, [1/5 1]);
G = tf(gamma, [1 beta alpha])    * Gmotor;

bode(G)
omega_n = sqrt(alpha);
ksi = beta/2/omega_n;

omega_r = omega_n * sqrt(1-ksi^2);
freq = omega_r/(2*pi);

freq_rad = [0.10, 0.60, 1.20, 1.70, 2.10, 2.50, 2.75, 3.00, 3.15, 3.30, 3.45, 3.60, 3.75, 3.85, 3.95, 4.00, 4.05, 4.10, 4.15, 4.25, 4.40, 4.70, 5.00, 8.00, 10.0, 25, 40, 60];


%curva estatica
yss = deg2rad(30);
uss = 34.2003*yss + 1.1513; 

A = 5; %amplitude de oscilação em U
dyss = rad2deg(A/34.2003); %amplitude de oscilação em graus


