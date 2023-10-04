clc, clear, close all

alpha = 23.24;
beta =  1.96;
gamma = 0.56;

Gmotor = tf(1, [1/5 1]);
G1 = tf(gamma, [1 beta alpha*cosd(15)])    * Gmotor;
G2 = tf(gamma, [1 beta alpha*cosd(30)])    * Gmotor;
G3 = tf(gamma, [1 beta alpha*cosd(40)])    * Gmotor;
G4 = tf(gamma, [1 beta alpha*cosd(50)])    * Gmotor;
G5 = tf(gamma, [1 beta alpha*cosd(60)])    * Gmotor;
G6 = tf(gamma, [1 beta alpha*cosd(70)])    * Gmotor;


Kf1 = 1 / dcgain(G1);
Kf2 = 1 / dcgain(G2);
Kf3 = 1 / dcgain(G3);
Kf4 = 1 / dcgain(G4);
Kf5 = 1 / dcgain(G5);
Kf6 = 1 / dcgain(G6);

C1 = tf(331.77 * conv([1 2.271], [1 0.1107]), conv([1 15], [1 15]));
C2 = tf(188.04 * conv([1 0.4792], [1 1.598]), conv([1 12], [1 12]));
C3 = tf(130.8 * conv([1 0.5912], [1 1.862]), conv([1 12], [1 12]));
C4 = tf(111.48 * conv([1 0.5075], [1 2.545]), conv([1 12], [1 12]));
C5 = tf(128.93 * conv([1 0.339], [1 3.29]), conv([1 12], [1 12]));
C6 = tf(97.451 * conv([1 0.2676], [1 5.522]), conv([1 12], [1 12]));