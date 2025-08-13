clc
clear 
close all

v_max = 20; % m/s
N = 1000; t = ([1:N]-1)/N; 
vH = v_max*[cos(2*pi*t);sin(2*pi*t)];

L = 2.5; theta = 0;
Mcin = [cos(theta), -L*sin(theta);...
        sin(theta),  L*cos(theta)];
    
w_max = v_max/L;
    
VW = (Mcin^-1)*vH;

VW(1,:) = (v_max-(abs(VW(2,:))/w_max)*v_max).*sign(VW(1,:));
vH2 = Mcin*VW; 

figure
plot(vH(1,:),vH(2,:))
hold on
plot(vH2(1,:),vH2(2,:),'r')

% Checando se bateu com as restrições
VW2 = (Mcin^-1)*vH2;
v_comp = (v_max-(abs(VW2(2,:))/w_max)*v_max).*sign(VW2(1,:));

e = (v_comp-VW2(1,:))*(v_comp-VW2(1,:))'