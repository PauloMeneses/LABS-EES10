clear all;

load("40_percent.mat")
plot(out.simout_30.Time, out.simout_30.Data(1:end,2));
grid on 

tp40 = 20.75 - 20.117;
yp40 = 52.245 - 43.155;
yss40 = 48.69 - 43.155;

Mp40 = (yp40-yss40)/yss40;

ksi40 = -log(Mp40)/sqrt(pi^2 + (log(Mp40))^2);
omega_n40 = pi/(tp40*sqrt(1-ksi40^2));
k40 = deg2rad(yss40)/(30-25.16); %amplitude do degrau a ser preenchida.
% 
% coloca o final menos o que estava antes, verificar o arquivo _20
%

alfa40  = omega_n40^2/cos(deg2rad(40))
beta40  = 2*ksi40*omega_n40
gamma40 = k40 * omega_n40^2