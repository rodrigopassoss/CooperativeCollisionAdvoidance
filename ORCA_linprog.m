function [v_new] = ORCA_linprog(vA_opt,vA_pref,vA_max,u,n,idx)

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
    %---

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
    %---
    
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
    %---

    % Junta tudo
    A_total = [A_abs; A_orca; A_circle];
    b_total = [b_abs; b_orca; b_circle];

    % Bounds (opcional, pode ser -Inf/Inf)
    lb = [-Inf; -Inf; 0; 0];  % t1 e t2 ≥ 0
    ub = [];
    
    x = linprog(f, A_total, b_total, [], [], lb, ub);
    if(isempty(x)) % Caso não tenha solução
        % x = 0.5*vA_opt; % Essa solução da problema pois os robôs esperam uma ação reciproca
        % O artigo propõe uma solução mais elaborada para isso!   
        x = ORCA_linprog2(vA_opt,vA_max,u,n);
%         plot_ORCA([A_orca;A_circle],[b_orca;b_circle],x(1:2),vA_pref,idx,nB);
%         pause;
    end
    
    % Debug
    if idx==1
        plot_ORCA([A_orca;A_circle],[b_orca;b_circle],x(1:2),vA_pref,idx,nB);
    end
%     if idx==1, plot_ORCA([A_orca],[b_orca],x(1:2),vA_pref,idx);end


    % Solução ótima de velocidade
    v_new = x(1:2);
end