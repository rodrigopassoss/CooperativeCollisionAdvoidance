function [r] = plot_VO(pA,pB,rA,rB,tau,p1,p2,v1,v2)
    C = (pB-pA)/tau;
    r = (rA+rB)/tau;
    theta1 = atan2(p1(2)-C(2), p1(1)-C(1));
    theta2 = atan2(p2(2)-C(2), p2(1)-C(1));
    if theta2<theta1
        theta2 = theta2+2*pi;
    end    
    angs = linspace(theta1,theta2,100);
    semiCirc = C + r*[cos(angs);sin(angs)];
    t = 0:100;
    ret1 = p1 + v1.*t;
    ret2 = p2 + v2.*t;
    figure(11)
    hold on
    grid on
    axis equal
    plot(ret1(1,:),ret1(2,:),'-b','lineWidth',3)
    plot(ret2(1,:),ret2(2,:),'-b','lineWidth',3)
    plot(semiCirc(1,:),semiCirc(2,:),'-b','lineWidth',3)
%     xlim([-20 5])

end