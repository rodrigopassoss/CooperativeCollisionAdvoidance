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
%     I=2; % Arbitrando um lado - convenção social
    P_borda = pontos(:,I);
    u = P_borda-P;
    % Obtenção do vetor n
    if I==1
        n = (P_borda-pAB/tau)/norm(P_borda-pAB/tau);
        %--- Caso tenha simetria
%         aux = acos((u/norm(u))'*(pAB/norm(pAB)));
%         if  aux == 0 || aux == pi
%             rot_angle = pi/2;
%             n = [cos(rot_angle) sin(rot_angle);-sin(rot_angle) cos(rot_angle)]*n;
%         end     
        %---
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
    
end