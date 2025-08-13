function [v_new]=ORCA(R,P,V,tau,vA_pref,vA_max,idx)
    rA = R(:,idx);
    pA = P(:,idx);
    vA_opt = V(:,idx);
    N=size(P,2);
    u = []; n = [];

    for i = setdiff(1:N,idx)
        rB = R(:,i);
        pB = P(:,i);
        vB_opt = V(:,i);
        % Definição de VO_(A|B)
        [p1,p2,v1,v2] = criar_VO(pA,pB,rA,rB,tau);
        % Obtenção dos vetores
        [u_,n_] = get_orca_vecs(vA_opt,vB_opt,pA,pB,rA,rB,tau,p1,p2,v1,v2);
        u = [u,u_];
        n = [n,n_];
    end

    %% Otimização - ORCA
    [v_new] = ORCA_linprog(vA_opt,vA_pref,vA_max,u,n,idx);

end