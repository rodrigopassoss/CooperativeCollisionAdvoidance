function v_new = ORCA_linprog2(vA_opt,vA_max,u,n)
    %Solução usando busca exaustiva

    N = 1e3; %Número de Partículas
    theta = rand(1,N);
    V = vA_max*rand(1,N).*[cos(2*pi*theta);sin(2*pi*theta)];
    d_AB = zeros(1,N);
    for i = 1:N
        d_AB(i) = max(dot((vA_opt+0.5*u)-V(:,i),n));
    end
    [~,I] = min(d_AB);
    v_new = V(:,I);

end