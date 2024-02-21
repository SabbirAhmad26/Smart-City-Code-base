xU = [60, 0, 0, 20];
x1 =[50, 4, 0, 29];
u1 = 0;
phi_1 = 0;
xC = [40, 0, 0, 25];
uC = 0;
phi_C = 0;
% output = my_func_for_test(xU,xC,x1,uC,phi_C,u1,phi_1);
phi_1 = 0;
phi_H = 0;
uH = 0;
h1 = 0;
h2 = 0;
h3 = 0;
h4 = 0;
xH = [1000, 4, 0, 25];
yH = [1000, 4, 0, 25];
eps1 = 0;
eps2 = 0;
eps3 = 0;
eps4 = 0;
sigma1 = 1;
sigma2 = 1;


vector = event_solve_qp (xU,xC,xH,x1,uC,phi_C,u1,phi_1,uH,phi_H,...
    h1,h2,h3,h4,eps1,eps2,eps3,eps4,sigma1,sigma2,yH);
