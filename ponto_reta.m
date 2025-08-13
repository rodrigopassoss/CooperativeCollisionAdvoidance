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


