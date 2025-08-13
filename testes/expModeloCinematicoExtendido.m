clc
clear 
close all

addpath D:\Documents\Arquivos_Pesquisa\codigos\simulador\orca

v_max = 20; % cm/s


L = 2.5; % (para usar o raciocínio do modelo cinemático extendido basta somar ao raio o comprimento L)
P = [0;0;0]

v = 20;
thetaH = pi* 90 /180;
vH = v*[cos(thetaH);sin(thetaH)];

tamos = 0.005;
duracao = 2;
N = round(duracao/tamos);
err_reg = 0;
tempo = 0;

VW_reg = [];

for i = 1:N
    
Mcin = [cos(P(3,end)), -L*sin(P(3,end));...
        sin(P(3,end)),  L*cos(P(3,end))];
    
VW = (Mcin^-1)*vH;
VW_reg = [VW_reg VW];

Mcin2 = [cos(P(3,end)) 0;sin(P(3,end)) 0;0 1];
P_curr = P(:,end) + (Mcin2*VW)*tamos;
P = [P P_curr];

[err, ~] = ponto_reta(P_curr(1:2), [0;0], vH/norm(vH));
err_reg(i+1) = err;
tempo(i+1) = i*tamos;    
end

figure(1)
quiver(0,0,vH(1),vH(2),'--b','linewidth',1)
hold on
plot(P(1,:),P(2,:),'k','linewidth',2)
axis equal

figure(2)
hold on
plot(tempo,err_reg)

figure(3)
subplot(211)
plot(VW_reg(1,:))
subplot(212)
plot(VW_reg(2,:))
