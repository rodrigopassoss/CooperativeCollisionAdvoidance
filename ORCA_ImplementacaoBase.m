clc
clear
close all



%% Cenário com dois robôs homogênos: A e B
PHI = 0
rA = 10; pA = [25*cosd(0+PHI);25*sind(0+PHI)]; 
rB = 10; pB = [25*cosd(180+PHI);25*sind(180+PHI)]; 
% Máxima Velocidade Holonômica
vA_max = 10;
vA_opt = vA_max*(pB-pA)/norm(pB-pA);
vB_opt = vA_max*(pA-pB)/norm(pA-pB);
%vB_opt = vA_max*(pA-pB+[5;5])/norm(pA-pB+[5;5]);
% Horizonte de tempo
tau = 2;
% Velocidade preferida
vA_pref = vA_opt;
%% Definição de VO_(A|B)
[p1,p2,v1,v2] = criar_VO(pA,pB,rA,rB,tau);
%% Plot do Velocity Obstacle:
plot_VO(pA,pB,rA,rB,tau,p1,p2,v1,v2);
%% Obtenção dos vetores
[u,n] = get_orca_vecs(vA_opt,vB_opt,pA,pB,rA,rB,tau,p1,p2,v1,v2);
%% Otimização - ORCA
tic
[v_new] = ORCA_linprog(vA_opt,vA_pref,vA_max,u,n);
toc
%% Plot da nova velocidade
hold on
quiver(0,0,vA_pref(1),vA_pref(2),'-r','linewidth',2)
quiver(0,0,v_new(1),v_new(2),'-g','linewidth',2)
xlabel('v_x','Interpreter', 'tex')
ylabel('v_y','Interpreter', 'tex')

%% Plot do ORCA
% Normal e ponto de fronteira
n = n';               % normal do semiplano (negativa de n_i)
norm_n = n / norm(n);
p = norm_n * (dot(vA_opt + 0.5 * u, n));      % ponto da linha: onde a restrição é ativa

% Vetor ortogonal para desenhar a linha
t = [-norm_n(2), norm_n(1)];

% Gera dois pontos para traçar a linha da restrição
pts = [p + 20*t; p - 20*t];
plot(pts(:,1), pts(:,2), 'k-', 'LineWidth', 1.5)
% Faixa (como um polígono paralelo à linha da restrição)
width = 1;  % espessura da faixa
v1 = p + width * norm_n;
v2 = p - 0 * norm_n;
quad = [v1 + 20*t; v1 - 20*t; v2 - 20*t; v2 + 20*t];
fill(quad(:,1), quad(:,2), [0.7 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
xlim([-40 20])
ylim([-6 36])

%% Plot dos Robôs
N = 0:100;
xc = cos(2*pi*[N]/100);
yc = sin(2*pi*[N]/100);
posA = pA + rA*[xc;yc];
posB = pB + rB*[xc;yc];
figure()
hold on
axis equal
text(pA(1),pA(2),'A')
ang = atan2(vA_opt(2),vA_opt(1));
quiver(pA(1)+rA*cos(ang),pA(2)+rA*sin(ang),vA_opt(1),vA_opt(2),'r','linewidth',2)
quiver(pA(1)+rA*cos(ang),pA(2)+rA*sin(ang),v_new(1),v_new(2),'g','linewidth',2)
plot(posA(1,:),posA(2,:),'k','linewidth',3)
text(pB(1),pB(2),'B')
ang = atan2(vB_opt(2),vB_opt(1));
quiver(pB(1)+rB*cos(ang),pB(2)+rB*sin(ang),vB_opt(1),vB_opt(2),'r','linewidth',2)
plot(posB(1,:),posB(2,:),'k','linewidth',3)
xlabel('x')
ylabel('y')





%% Funções Auxiliares
function [dist, ponto] = ponto_arco(P, C, r, p1, p2)
    theta1 = atan2(p1(2)-C(2), p1(1)-C(1));
    theta2 = atan2(p2(2)-C(2), p2(1)-C(1));
    
    % Vetor do centro ao ponto
    v = P - C;
    
    % Ângulo entre vetor CP e o eixo x
    thetaP = atan2(v(2), v(1));
    
%     % Normalizar ângulos para [0, 2*pi)
%     theta1 = mod(theta1, 2*pi);
%     theta2 = mod(theta2, 2*pi);
%     thetaP = mod(thetaP, 2*pi);

    % Verifica se thetaP está dentro do arco
    if theta1 < theta2
        dentro = (thetaP >= theta1) && (thetaP <= theta2);
    else
        dentro = (thetaP >= theta1) || (thetaP <= theta2);
    end

    if dentro
        % Ponto mais próximo está sobre o arco
        ponto = C + r * [cos(thetaP); sin(thetaP)];
    else
        % Avaliar os dois extremos
        A1 = C + r * [cos(theta1); sin(theta1)];
        A2 = C + r * [cos(theta2); sin(theta2)];
        d1 = norm(P - A1);
        d2 = norm(P - A2);
        if d1 < d2
            ponto = A1;
        else
            ponto = A2;
        end
    end

    % Distância entre P e o ponto mais próximo no arco
    dist = norm(P - ponto);
end

function [dist, ponto] = ponto_reta(P, P0, v)
    

    % Vetor do início da reta ao ponto P
    w = P - P0;

    % Projeção escalar de w sobre v
    t = dot(w, v);

    if t < 0
        % Antes do início do segmento
        ponto = P0;
    else
        % Projeção dentro do segmento
        ponto = P0 + t * v;
    end

    % Distância entre P e o ponto mais próximo
    dist = norm(P - ponto);
end

function [r] = plot_VO(pA,pB,rA,rB,tau,p1,p2,v1,v2)
    C = (pB-pA)/tau;
    r = (rA+rB)/tau;
    theta1 = atan2(p1(2)-C(2), p1(1)-C(1));
    theta2 = atan2(p2(2)-C(2), p2(1)-C(1));
    if theta2<theta1
        theta2 = theta2+2*pi;
    end    
    angs = linspace(theta1,theta2,100);
    semiCirc = C + r*[cos(angs);sin(angs)];
    t = 0:100;
    ret1 = p1 + v1.*t;
    ret2 = p2 + v2.*t;
    
    figure(1)
    hold on
    grid on
    axis equal
    plot(ret1(1,:),ret1(2,:),'-b','lineWidth',3)
    plot(ret2(1,:),ret2(2,:),'-b','lineWidth',3)
    plot(semiCirc(1,:),semiCirc(2,:),'-b','lineWidth',3)
%     xlim([-20 5])
end

function [u,n] = get_orca_vecs(vA_opt,vB_opt,pA,pB,rA,rB,tau,p1,p2,v1,v2)
    P = vA_opt-vB_opt;
    pAB = pB-pA;
    
    P = 0.88*P + 0.05*norm(P)*[-v1(2);v1(1)] + 0.07*norm(P)*[-v2(2);v2(1)];
    [dist0, ponto0] = ponto_arco(P, pAB/tau, (rB+rA)/tau, p1, p2);
    [dist1, ponto1] = ponto_reta(P, p1, v1);
    [dist2, ponto2] = ponto_reta(P, p2, v2);

    dists = [dist0,dist1,dist2];
    pontos = [ponto0,ponto1,ponto2];
    [~,I] = min(dists);
%     I = 2;
    P_borda = pontos(:,I);
    u = P_borda-P;
    % Obtenção do vetor n
    if I==1
        n = (P_borda-pAB/tau)/norm(P_borda-pAB/tau);
    elseif I==2
        n = [-v1(2);v1(1)]; 
        if dot((pAB/tau - P_borda),n)>0 % se sim, o normal aponda para dentro do VO
            n=-n;
        end
    else
        n = [-v2(2);v2(1)]; 
        if dot((pAB/tau - P_borda),n)>0 % se sim, o normal aponda para dentro do VO
            n=-n;
        end
    end
    
    % Plot dos vetores
    quiver(P(1),P(2),u(1),u(2),'g')
    text(P(1)+u(1)*0.5,P(2)+u(2)*0.5,' u')
    quiver(P(1)+u(1),P(2)+u(2),n(1),n(2),'b')
    text(P(1)+u(1)+0.5*n(1),P(2)+u(2)+0.5*n(2),' n')
    plot(P(1),P(2),'.r','MarkerSize',15)
%     quiver(0,0,vA_opt(1),vA_opt(2),'k','linewidth',2)
    quiver(vA_opt(1),vA_opt(2),u(1)*0.5,u(2)*0.5,'g')
    plot(vA_opt(1),vA_opt(2),'.g','MarkerSize',15)
    text(vA_opt(1),vA_opt(2),'    V^{opt}_A','Interpreter', 'tex')
    plot(P_borda(1),P_borda(2),'.y','MarkerSize',15)

end

function [p1,p2,v1,v2] = criar_VO(pA,pB,rA,rB,tau)
    pAB = pB-pA;
    alfa = asin((rA+rB)/norm(pAB));
    beta = atan2(pAB(2),pAB(1));
    % Vetores diretores das retas
    v1 = [cos(beta+alfa);sin(beta+alfa)]; 
    v2 = [cos(beta-alfa);sin(beta-alfa)];
    % Interseção das retas com o circulo
    p1 = (cos(alfa)*norm(pAB)/tau)*v1;
    p2 = (cos(alfa)*norm(pAB)/tau)*v2;
end

function [v_new] = ORCA_linprog(vA_opt,vA_pref,vA_max,u,n)

    % Variáveis de decisão: [vx; vy; t1; t2]
    f = [0; 0; 1; 1];  % Minimizar t1 + t2

    % Restrição de valor absoluto
    % t1 >= vx - vAx_pref
    % t1 >= -(vx - vAx_pref)
    % t2 >= vy - vAy_pref
    % t2 >= -(vy - vAy_pref)

    A_abs = [
        1  0 -1  0;
       -1  0 -1  0;
        0  1  0 -1;
        0 -1  0 -1
    ];

    b_abs = [
        vA_pref(1);
       -vA_pref(1);
        vA_pref(2);
       -vA_pref(2)
    ];

    % Restrição ORCA: A_orca * [vx; vy] <= b_orca
    nB=size(n,2); % Quantidade de robôs B
    A_orca = [];
    b_orca = [];
    % Adiciona todas as restrições devido a presença de outros robôs
    for i=1:nB
        a = -n(:,i)';  % linha
        b_val = -dot(vA_opt + 0.5 * u(:,i), n(:,i));
        A_orca = [A_orca;a];
        b_orca = [b_orca;b_val];  
    end
    A_orca = [A_orca, zeros(size(A_orca,1), 2)];

    
    % Restrições de Velocidade
    N = 16;  % mais lados = aproximação melhor
    theta = linspace(0, 2*pi, N+1);
    theta(end) = [];  % remover duplicata

    A_circle = zeros(N, 4);
    b_circle = vA_max * ones(N, 1);

    for i = 1:N
        dir = [cos(theta(i)), sin(theta(i))];  % vetor direção do lado do polígono
        A_circle(i, 1:2) = dir;
    end
    
    % Junta tudo
    A_total = [A_abs; A_orca; A_circle];
    b_total = [b_abs; b_orca; b_circle];

    % Bounds (opcional, pode ser -Inf/Inf)
    lb = [-Inf; -Inf; 0; 0];  % t1 e t2 ≥ 0
    ub = [];
    
    [x,fval] = linprog(f, A_total, b_total, [], [], lb, ub)

    % Solução ótima de velocidade
    v_new = x(1:2);
end




