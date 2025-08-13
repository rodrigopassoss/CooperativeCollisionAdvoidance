clc
clear 
close all

%Debug
% data=load('temp.mat');

N = 2; % Número de Robôs
v_max = 20; % 10 m/s
R = 5*ones(1,N); % Raio dos robôs
PHI = pi* 0 / 180;
P_init = 200*[cos(2*pi*([1:N] -1)/N + PHI);sin(2*pi*([1:N] -1)/N + PHI)];% + randi(200,2,N);
% P_init = data.P_init;
P_goal = 200*[cos(2*pi*([1:N] -1)/N - pi  + PHI);sin(2*pi*([1:N] -1)/N - pi  + PHI)];
V = P_goal-P_init;
for i = 1:N
    V(:,i) = v_max*V(:,i)/norm(V(:,i));
end
V_pref = V;
V_new = V;

P = P_init;
Traj_x = P(1,:)';
Traj_y = P(2,:)';
Vel_x = V(1,:)';
Vel_y = V(2,:)';

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
    p2(k) = plot(Traj_x(k,:),Traj_y(k,:),'-k','linewidth',2);
end

tamos = 0.1;
d = 10e10;
t = 0;
T = [];
while d > R(1)
    
    t = t + tamos;
    V=V_new;
    for i = 1:N
        % Atualiza o V_pref
        V_pref(:,i) = (P_goal(:,i)-P(:,i));
        if norm(V_pref(:,i))>v_max % Satura caso seja maior que o v_max
            V_pref(:,i) = v_max*V_pref(:,i)/norm(V_pref(:,i));
        end
        % Calcula a nova velocidade
        tic
        v_new=ORCA(R+5,P,V,2,V_pref(:,i),v_max,i);
        T = [T toc];
        V_new(:,i) = v_new;
    end
    % Atualiza posições
    P = P + V_new*tamos;

    % Condição de parada
    aux = P_goal-P;
    d = max(sqrt(aux(1,:).^2 + aux(2,:).^2));

    % Atualiza trajetória
    Traj_x = [Traj_x P(1,:)'];
    Traj_y = [Traj_y P(2,:)'];
    Vel_x = [Vel_x V_new(1,:)'];
    Vel_y = [Vel_y V_new(2,:)'];

    % Realização do Plot
    for k = 1:N
        xc = [cosd([0:360])].*R(k);
        yc = [sind([0:360])].*R(k);
        xc1 = xc+P(1,k);
        yc1 = yc+P(2,k);
        set(p0(k),'xdata',xc1,'ydata',yc1);
        set(p1(k),'xdata',xc1,'ydata',yc1);
        set(p2(k),'xdata',Traj_x(k,:),'ydata',Traj_y(k,:));
    end
    drawnow;

end
display(['ORCA custou: ',num2str(mean(T)*1000),'ms'])

% figure(2)
% subplot(211)
% hold on 
% plot(Vel_x')
% ylabel('vx')
% subplot(212)
% hold on 
% plot(Vel_y')
% ylabel('vy')


