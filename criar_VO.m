function [p1,p2,v1,v2] = criar_VO(pA,pB,rA,rB,tau)
    pAB = pB-pA;
    alfa = asin((rA+rB)/norm(pAB));
    if imag(alfa)~=0
         display('colidiu 1')
%         figure(1)
%         hold on
%         plot([pA(1);pB(1)],[pA(2);pB(2)],'-r','linewidth',4)
%         pause;
        alfa = asin(((rA+rB)-5)/norm(pAB));
    end
    beta = atan2(pAB(2),pAB(1));
    % Vetores diretores das retas
    v1 = [cos(beta+alfa);sin(beta+alfa)]; 
    v2 = [cos(beta-alfa);sin(beta-alfa)];
    % Interseção das retas com o circulo
    p1 = (cos(alfa)*norm(pAB)/tau)*v1;
    p2 = (cos(alfa)*norm(pAB)/tau)*v2;
end