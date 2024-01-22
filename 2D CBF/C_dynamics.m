function dx = C_dynamics(t,x)
Lw = 4;
dx = zeros(6,1);
dx(1) = x(4)*cos(x(3))-x(4)*sin(x(3))*x(6);
dx(2) = x(4)*sin(x(3))+x(4)*cos(x(3))*x(6);
dx(3) = x(4)/Lw*x(6);
dx(4) = x(5);
dx(5) = 0; % uC
dx(6) = 0; % phiC
end