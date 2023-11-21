%clc, clear, close all
s = tf('s');

alpha = 18.02;
beta =  1.74;
gamma = 0.5;

Gmotor = tf(1, [1/5 1]);
G = tf(gamma, [1 beta alpha])    * Gmotor;


ksi = 0.7;
omegan = 2;

C1 = 18*(s+5)*(s^2 + 1.74*s + 18.82)/(s*(s+18)*(s^2+2*ksi*omegan*s+omegan^2));

step(C1*G/(1+C1*G))
