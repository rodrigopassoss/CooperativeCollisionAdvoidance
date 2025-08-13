function [r] = plot_ORCA(A,b,v_opt,vA_pref,idx,nB)

% Número de restrições
m = size(A,1);

% Domínio para visualização
xrange = linspace(-40, 40, 500);
yrange = linspace(-40, 40, 500);
[X, Y] = meshgrid(xrange, yrange);
Z = zeros(size(X));

% Construir região factível
feasible = true(size(X));
for i = 1:m
    feasible = feasible & (A(i,1)*X + A(i,2)*Y <= b(i));
end

% Plot da região viável
figure(20+idx); axis equal
contourf(X, Y, feasible, [1 1], 'k', 'LineWidth', 1.5)
hold on;
colormap([0.9 0.5 0.5])  % fundo cinza claro
alpha(0.2)

% Plot das faixas ORCA (fronteiras)
for i = 1:nB
    % Normal e ponto de fronteira
    n = -A(i, 1:2);               % normal do semiplano (negativa de n_i)
    norm_n = n / norm(n);
    p = norm_n * (-b(i));      % ponto da linha: onde a restrição é ativa

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
end

% Origem
plot(0, 0, 'ko', 'MarkerFaceColor', 'k')
quiver(0, 0, vA_pref(1), vA_pref(2), 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5)
quiver(0, 0, v_opt(1), v_opt(2), 0, '--r', 'LineWidth', 2, 'MaxHeadSize', 0.5)




% Estética
xlabel('v_x','Interpreter', 'tex')
ylabel('v_y','Interpreter', 'tex')
title('Regiões ORCA a partir de A · v ≤ b','Interpreter', 'tex')
grid on
xlim([-40 40])
ylim([-40 40])
drawnow
hold off


end