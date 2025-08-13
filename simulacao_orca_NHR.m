clc
clear 
close all

A = imread('mapa2.png');
A = A(:,:,1);


%Simuação do ORCA para robôs não-holonômicos

N = 2; % Número de Robôs
v_max = 20; % 20 cm/s
R = 5*ones(1,N); % Raio dos robôs
PHI = pi* 45 / 180;
P_init = [500;500] + 400*[cos(2*pi*([1:N] -1)/N + PHI);sin(2*pi*([1:N] -1)/N + PHI)];% + randi(200,2,N);
P_goal = [500;500] + 400*[cos(2*pi*([1:N] -1)/N - pi  + PHI);sin(2*pi*([1:N] -1)/N - pi  + PHI)];
V = P_goal-P_init;
THETA = atan2(V(2,:),V(1,:));
for i = 1:N
    V(:,i) = v_max*V(:,i)/norm(V(:,i));
end
V_pref = V;
V_new = V;

P = P_init;
Poses = [P; THETA]; 
for i = 1:N
    VW(:,i) = inv(matriz_cinematica(Poses(3,i),R(i)))*V_pref(:,i);
end
Traj_x = P(1,:)';
Traj_y = P(2,:)';
Traj2_x = Poses(1,:)';
Traj2_y = Poses(2,:)';

figure(1)
hold on
for k = 1:N
    transparency = 0.25;  % Altere conforme necessário
    fillColor = [0, 0, 0.5];  % Cor vermelho claro, você pode ajustar isso
    xc = [cosd([0:360])].*R(k);
    yc = [sind([0:360])].*R(k);
    xc1 = xc+P(1,k);
    yc1 = xc+P(2,k);
    p0(k) = patch(xc1, yc1, fillColor, 'EdgeColor', 'none', 'FaceAlpha', transparency);
    p1(k) = plot(xc1,yc1,'-','linewidth',2);
    p2(k) = plot(Traj_x(k,:),Traj_y(k,:),'--k','linewidth',1);
    p3(k) = plot(Traj2_x(k,:),Traj2_y(k,:),'-b','linewidth',2);
    p4(k) = plot(0,0,'.m','MarkerSize',15);
end

tamos = 0.1;
d = 10e10;
t = 0;
T = [];
VW_hist = []; dists = [];cnt = 0;
while d > R(1)
    cnt = cnt + 1;
    t = t + tamos;
    V=V_new;
    for i = 1:N
        % Atualiza o V_pref
        V_pref(:,i) = (P_goal(:,i)-Poses(1:2,i));
        if norm(V_pref(:,i))>v_max % Satura caso seja maior que o v_max
            V_pref(:,i) = v_max*V_pref(:,i)/norm(V_pref(:,i));
        end
        % Calcula a nova velocidade
        tic
        v_new=NH_ORCA_OBS(R*1.5,Poses,A,V,2,V_pref(:,i),v_max,i,p4(i));
%         v_new=NH_ORCA(R*1.5,Poses,V,2,V_pref(:,i),v_max,i);
        T = [T toc];
        V_new(:,i) = v_new;
    end
    
    % Atualiza posições - Holonômica
    P = P + V_new*tamos;
    % Atualiza posições - Não-Holonômica
    for i = 1:N % Converte a velocidade holonômica em não-holonomica
        VW(:,i) = inv(matriz_cinematica(Poses(3,i),R(i)*0.5))*V_new(:,i);
        Poses(:,i) = Poses(:,i) + [cos(Poses(3,i)) 0;sin(Poses(3,i)) 0;0 1]*VW(:,i)*tamos;
    end
    
    VW_hist(:,:,cnt) = VW;
    for i = 1:N
        for j = i:N
            if i ~= j
                dists(i,j,cnt) = norm(Poses(1:2,i)-Poses(1:2,j));
            end
        end
    end
    
   

    % Condição de parada
    aux = P_goal-Poses(1:2,:);
    d = max(sqrt(aux(1,:).^2 + aux(2,:).^2));

    % Atualiza trajetória
    Traj_x = [Traj_x P(1,:)'];
    Traj_y = [Traj_y P(2,:)'];
    Traj2_x = [Traj2_x Poses(1,:)'];
    Traj2_y = [Traj2_y Poses(2,:)'];

    % Realização do Plot
    for k = 1:N
        xc = [cosd([0:360])].*R(k);
        yc = [sind([0:360])].*R(k);
        xc1 = xc+Poses(1,k);
        yc1 = yc+Poses(2,k);
        set(p0(k),'xdata',xc1,'ydata',yc1);
        set(p1(k),'xdata',xc1,'ydata',yc1);
        set(p2(k),'xdata',Traj_x(k,:),'ydata',Traj_y(k,:));
        set(p3(k),'xdata',Traj2_x(k,:),'ydata',Traj2_y(k,:));
    end
    drawnow;

end
display(['ORCA custou: ',num2str(mean(T)*1000),'ms'])


function [M] = matriz_cinematica(theta,L)
        M = [cos(theta), -L*sin(theta);...
             sin(theta),  L*cos(theta)];
end

