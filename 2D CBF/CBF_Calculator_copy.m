clear all
close all
clc
syms x0
syms y0
syms psi0
syms v0
syms k1;
syms k2;
syms k3;
syms k4;
syms u;
syms phi;
syms k;
syms psi;
syms Rl;
syms r;
syms xo;
syms yo;
syms xcl;
syms ycl;
syms lr;
syms lf;
syms xd;
syms psi_d
syms y_d
%%% road


States = [x0,y0,psi0,v0];


Psi1 = 0;
Psi2_Lf = 0;
Psi2_Lg_u = 0;
Psi2_Lg_psi = 0;

f = [v0*cos(psi0);v0*sin(psi0);0;0];
g = [0,0;0,0;v0/(lf + lr),0;0,1];
Control = [psi;u];

% Psi0 = (x0 - xo)^2+(y0 - yo)^2 - r^2;
% 
% Psi0 = (psi0 - psi_d)^2;
% 
% Psi0 = (y0 - y_d)^2;

% Psi0 = -(x0 - xcl)^2-(y0 - ycl)^2  +2*Rl*sqrt((x0 - xcl)^2+(y0 - ycl)^2) - Rl^2;

% Psi0 = sqrt((x0 - xcr)^2+(y0 - ycr)^2)  - Rr;

Psi0 = -sqrt((x0 - xcl)^2+(y0 - ycl)^2)  + Rl;

for i = 1:1:length(States)
    Psi1 = Psi1 + diff(Psi0,States(i))*(f(i)+g(i,:)*Control);
end
Psi1 = simplify(Psi1) + k1*Psi0;
for i = 1:1:length(States)
    Psi2_Lf = Psi2_Lf + diff(Psi1,States(i))*(f(i));
    Psi2_Lg_psi = Psi2_Lg_psi + diff(Psi1,States(i))*(g(i,1));
    Psi2_Lg_u = Psi2_Lg_u + diff(Psi1,States(i))*(g(i,2));
end

Psi2_Lf = simplify(Psi2_Lf) + k2*Psi1;


% syms r;
% syms a b c;
% Psi0_r = 0;
% Psi1_r = 0;
% psi2_r = 0;
% Psi2_r_Lf = 0;
% Psi2_r_Lg_u = 0;
% Psi2_r_Lg_phi = 0;
% 
% 
% 
% 
% Psi0_r = -(a*xH + b*yH + c)^2 + r^2*(a^2+b^2);
% for i = 1:1:length(States)
%     Psi1_r = Psi1_r + diff(Psi0_r,States(i))*(f(i)+g(i,:)*Control);
% end
% Psi1_r = simplify(Psi1_r) + k1*Psi0_r;
% for i = 1:1:length(States)
%     Psi2_r_Lf = Psi2_r_Lf + diff(Psi1_r,States(i))*(f(i));
%     Psi2_r_Lg_u = Psi2_r_Lg_u + diff(Psi1_r,States(i))*(g(i,1));
%     Psi2_r_Lg_phi = Psi2_r_Lg_phi + diff(Psi1_r,States(i))*(g(i,2));
% end
% 
% Psi2_r_Lf = simplify(Psi2_r_Lf) + k2*Psi1_r;


