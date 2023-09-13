clc, clear, close all

alpha = 23.24;
beta =  1.96;
gamma = 0.56;

Gmotor = tf(1, [1/5 1]);
G1 = tf(gamma, [1 beta alpha*cosd(15)]) * Gmotor;
G2 = tf(gamma, [1 beta alpha*cosd(37.5)]) * Gmotor;
G3 = tf(gamma, [1 beta alpha*cosd(55)]) * Gmotor;
G4 = tf(gamma, [1 beta alpha*cosd(70)]) * Gmotor;
G5 = tf(gamma, [1 beta alpha*cosd(85)]) * Gmotor;
G6 = tf(gamma, [1 beta alpha*cosd(100)]) * Gmotor;
G7 = tf(gamma, [1 beta alpha*cosd(117.5)]) * Gmotor;
G8 = tf(gamma, [1 beta alpha*cosd(145)]) * Gmotor;

% polo alocado em aproximadamente -2 e +- 1.86j

N_pid1 = 20; C1 = tf([1 1.264 19.01] *  68.264, [1 N_pid1 0]);
N_pid2 = 21; C2 = tf([1 2.03 16] *  79.76, [1 N_pid2 0]);
N_pid3 = 21; C3 = tf([1 1.9 11.6] *  73.51, [1 N_pid3 0]);
N_pid4 = 20; C4 = tf([1 1.94  7.2] *  68, [1 N_pid4 0]);
N_pid5 = 20; C5 = tf([1 2.1  2.03] *  64.86, [1 N_pid5 0]);

[Apid1, Bpid1, Cpid1, Dpid1] = tf2ss(C1.num{1}, C1.den{1});
[Apid2, Bpid2, Cpid2, Dpid2] = tf2ss(C2.num{1}, C2.den{1});
[Apid3, Bpid3, Cpid3, Dpid3] = tf2ss(C3.num{1}, C3.den{1});
[Apid4, Bpid4, Cpid4, Dpid4] = tf2ss(C4.num{1}, C4.den{1});
[Apid5, Bpid5, Cpid5, Dpid5] = tf2ss(C5.num{1}, C5.den{1});

% M * [Kp Ki Kd]' = [a b c]' from PID numerator a*s^2 + b*s + c
M = [1 0 C1.den{1}(2) ; C1.den{1}(2) 1 0 ; 0 C1.den{1}(2) 0];
gains = M\C1.num{1}';
Kp_pid1 = gains(1);
Ki_pid1 = gains(2);
Kd_pid1 = gains(3);

M = [1 0 C2.den{1}(2) ; C2.den{1}(2) 1 0 ; 0 C2.den{1}(2) 0];
gains = M\C2.num{1}';
Kp_pid2 = gains(1);
Ki_pid2 = gains(2);
Kd_pid2 = gains(3);

M = [1 0 C3.den{1}(2) ; C3.den{1}(2) 1 0 ; 0 C3.den{1}(2) 0];
gains = M\C3.num{1}';
Kp_pid3 = gains(1);
Ki_pid3 = gains(2);
Kd_pid3 = gains(3);

M = [1 0 C4.den{1}(2) ; C4.den{1}(2) 1 0 ; 0 C4.den{1}(2) 0];
gains = M\C4.num{1}';
Kp_pid4 = gains(1);
Ki_pid4 = gains(2);
Kd_pid4 = gains(3);

M = [1 0 C5.den{1}(2) ; C5.den{1}(2) 1 0 ; 0 C5.den{1}(2) 0];
gains = M\C5.num{1}';
Kp_pid5 = gains(1);
Ki_pid5 = gains(2);
Kd_pid5 = gains(3);

% create reference signal
sim('signal_generator');
t = simout.Time;
r = simout.Data;
clear simout tout
save ../step10.mat
