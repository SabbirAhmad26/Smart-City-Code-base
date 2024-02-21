function dx = real_dynamics(t,x)
Lw = 4;
dx = zeros(12,1);
dx(1) = x(4)*cos(x(3))*x(7) - x(4)*sin(x(3))*x(6) + x(9);
dx(2) = x(4)*sin(x(3))*x(8) + x(4)*cos(x(3))*x(6) + x(10);
dx(3) = x(4)/Lw*x(6) + x(11);
dx(4) = x(5) + x(12);
dx(5) = 0; % uH
dx(6) = 0; % phiH
dx(7) = 0; % sigma1
dx(8) = 0; % sigma2
dx(9) = 0; % eps1
dx(10) = 0; % eps2
dx(11) = 0; % eps3
dx(12) = 0; % eps4
end