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