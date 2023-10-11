clc, clear, close all

alpha = 21.8015;
beta =  2.0383;
gamma = 0.5602;

Gmotor = tf(1, [1/5 1]);
G = tf(gamma, [1 beta alpha])    * Gmotor;


%Raizes_G = roots(G.denominator{1});

%meu_polo = 2.47+1i*3;

%p1 = -12;
%p2 = -12;

Kf = 1 / dcgain(G);


C = tf(197.52 * conv([1 0.3005], [1 1.566]), conv([1 12], [1 12]));
