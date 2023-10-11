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


Raizes_G1 = roots(G1.denominator{1});
Raizes_G2 = roots(G2.denominator{1});
Raizes_G3 = roots(G3.denominator{1});
Raizes_G4 = roots(G4.denominator{1});
Raizes_G5 = roots(G5.denominator{1});
Raizes_G6 = roots(G6.denominator{1});

%meu_polo = 2.47+1i*3;

%p1 = -12;
%p2 = -12;






Kf1 = 1 / dcgain(G1);
Kf2 = 1 / dcgain(G2);
Kf3 = 1 / dcgain(G3);
Kf4 = 1 / dcgain(G4);
Kf5 = 1 / dcgain(G5);
Kf6 = 1 / dcgain(G6);

C1 = tf(204.68 * conv([1 1.273], [1 0.5525]), conv([1 12], [1 12]));
C2 = tf(185.39 * conv([1 0.4792], [1 1.598]), conv([1 12], [1 12]));
C3 = tf(172.85 * conv([1 0.5912], [1 1.862]), conv([1 12], [1 12]));
C4 = tf(147.24 * conv([1 0.4920], [1 2.476]), conv([1 12], [1 12]));
C5 = tf(121.65 * conv([1 0.5241], [1 3.296]), conv([1 12], [1 12]));
C6 = tf(133.34 * conv([1 1.9530], [1 3.182]), conv([1 12], [1 12]));