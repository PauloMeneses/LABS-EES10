%clc, clear, close all

alpha = 18.02;
beta =  1.74;
gamma = 0.5;

Gmotor = tf(1, [1/5 1]);
G = tf(gamma, [1 beta alpha])    * Gmotor;

%bode(G)
omega_n = sqrt(alpha);
ksi = beta/2/omega_n;

omega_r = omega_n * sqrt(1-ksi^2);
freq = omega_r/(2*pi);

freq_rad = [0.10, 0.60, 1.20, 1.70, 2.10, 2.50, 2.75, 3.00, 3.15, 3.30, 3.45, 3.60, 3.75, 3.85, 3.90, 3.95, 4.00, 4.05, 4.09, 4.12, 4.15, 4.20, 4.30, 4.45, 4.70, 5.00, 8.00, 10.0];

freq_hz  = 1/(2*pi) * freq_rad;


