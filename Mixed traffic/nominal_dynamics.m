function dx = nominal_dynamics(t,x)
Lw = 4;
dx = zeros(8,1);
dx(1) = dx(4)*cos(dx(3)) + x(5);
dx(2) = dx(4)*sin(dx(3)) + x(6);
dx(3) = x(4)/Lw + x(7);
dx(4) = x(8);
dx(5) = 0; % h1
dx(6) = 0; % h2
dx(7) = 0; % h3
dx(8) = 0; % h4
end