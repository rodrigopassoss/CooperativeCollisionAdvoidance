function [v_new]=NH_ORCA_OBS(R,P,A,V,tau,vA_pref,vA_max,idx,plt)
    rA = R(:,idx);
    pA = P(1:2,idx);
    vA_opt = V(:,idx);
    N=size(P,2);
    u = []; n = [];

    for i = setdiff(1:N,idx)
        rB = R(:,i);
        pB = P(1:2,i);
        vB_opt = V(:,i);
        % Definição de VO_(A|B)
        [p1,p2,v1,v2] = criar_VO(pA,pB,rA,rB,tau);
        % Obtenção dos vetores
        [u_,n_] = get_orca_vecs(vA_opt,vB_opt,pA,pB,rA,rB,tau,p1,p2,v1,v2);
        u = [u,u_];
        n = [n,n_];
    end
    % Consideração de obstáculos estáticos
    [s,s2,v_sensor,colidiu] = sensor_data(P(:,idx),rA,linspace(0,2*pi,72),100,A);
    I = find(v_sensor<(vA_max*tau));
    for i = 1:length(I)
        % Definição de VO_(A|B)
        [p1,p2,v1,v2] = criar_VO(pA,s2(:,I(i)),rA,0,tau);
        % Obtenção dos vetores
        [u_,n_] = get_orca_vecs(vA_opt,0,pA,s2(:,I(i)),rA,0,tau,p1,p2,v1,v2); 
        u = [u,2*u_];
        n = [n,n_];
    end
%     if idx==1
        set(plt,'xdata',s2(1,:),'ydata',s2(2,:))
%     end

    %% Otimização - NH_ORCA
    thetaA = P(3,idx);
    [v_new] = NH_ORCA_linprog(vA_opt,vA_pref,vA_max,thetaA,u,n,idx);

end