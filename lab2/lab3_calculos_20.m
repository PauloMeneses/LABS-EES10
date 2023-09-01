clear all;
load("20_percent.mat")
plot(out.simout_30.Time, out.simout_30.Data(1:end,2));

tp20 = 20.793 - 20.024;
yp20 = 24.705 - 19.575;
yss20 = 23.355 - 19.575;

Mp20 = (yp20-yss20)/yss20;

ksi20 = -log(Mp20)/sqrt(pi^2 + (log(Mp20))^2);
omega_n20 = pi/(tp20*sqrt(1-ksi20^2));
k20 = deg2rad(yss20/(15-13.03)); %amplitude do degrau de entrada eh (15-13.03)

alfa20  = omega_n20^2/cos(deg2rad(20))
beta20  = 2*ksi20*omega_n20
gamma20 = k20 * omega_n20^2