function [v_new] = NH_ORCA_linprog(vA_opt,vA_pref,vA_max,thetaA,u,n,idx)

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
    
    % Restrições de velocidade de definida a partir do MCE
    thetaA = thetaA-pi/4;
    A_nh = [
       cos(thetaA),  sin(thetaA), 0, 0;
      -cos(thetaA), -sin(thetaA), 0, 0;
       sin(thetaA), -cos(thetaA), 0, 0;
      -sin(thetaA),  cos(thetaA), 0, 0;
    ];

    b_nh = [
       vA_max;
       vA_max;
       vA_max;
       vA_max;
    ]/sqrt(2);
    %---

    % Junta tudo
    A_total = [A_abs; A_orca; A_nh];
    b_total = [b_abs; b_orca; b_nh];

    % Bounds (opcional, pode ser -Inf/Inf)
    lb = [-Inf; -Inf; 0; 0];  % t1 e t2 ≥ 0
    ub = [];
    
    x = linprog(f, A_total, b_total, [], [], lb, ub);
    if(isempty(x)) % Caso não tenha solução
        % Caso não ache solução desconsidera a restrição não holonômica
        display('Desconsiderando - NH')
        x = vA_opt*0.5;
%         x = ORCA_linprog(vA_opt,vA_pref,vA_max,u,n,idx)*0.5;
%         x = ORCA_linprog2(vA_opt,vA_max,u,n);
%         plot_ORCA([A_orca;A_circle],[b_orca;b_circle],x(1:2),vA_pref,idx,nB);
%         pause;
    end
    
    % Debug
    if idx==1
%         plot_ORCA([A_orca;A_nh],[b_orca;b_nh],x(1:2),vA_pref,idx,nB);
    end
%     if idx==1, plot_ORCA([A_orca],[b_orca],x(1:2),vA_pref,idx);end


    % Solução ótima de velocidade
    v_new = x(1:2);
end