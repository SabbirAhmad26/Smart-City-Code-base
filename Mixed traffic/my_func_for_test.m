function vector = my_func_for_test (xU,xC,x1,uC,phi_C,u1,phi_1)

%% initial settings
Lw = 4;
lanewidth = 4;

% weights of time and energy
v_des = 30;
v_min = 15;  v_max = 35;
u_min = -7;  u_max = 3.3;

% safety ellipse
aH = 0.6; bH = 0.1;
aC = 0.6; bC = 0.15;

% global U
dt = 0.05;
vU=20;


% events and error bounds
s_xc = 0.01; s_yc = 0.005; s_vc = 1; s_thetaC = 0.01;
s_xH = 0.01; s_yH = 0.005; s_vH = 1; s_thetaH = 0.01;
s_x1 = 0.01; s_y1 = 0.005; s_v1 = 1; s_theta1 = 0.01;
s_xU = 0.01; s_yU = 0.005; s_vU = 1; s_thetaU = 0.01;
ex_b = 0.2; ey_b = 0.1; ev_b = 1;
ex_dot_b = 0.5; ey_dot_b = 0.2; ev_dot_b = 1;

% CBF parameters
k1 = 1;
k2 = 1;
k3 = 1;
k4 = 10;
k5 = 1;
k6 = 1;
k7 = 1;
k8 = 1;
k9 = 1;
k10 = 1;
k11 = 1;
k12 = 1;
k13 = 1;
kU = 10;

% CLF parameters
vareps1 = 1;
vareps2 = 1;
vareps3 = 1;
vareps6 = 1;




% find the minimum terms
% vector = x,y,theta,v,ex,ey,etheta,ev,ex_dot,ey_dot,etheta_dot,ev_dot
xC_tk = xC(1); yC_tk = xC(2); thetaC_tk = xC(3); vC_tk = xC(4);
x1_tk = x1(1); y1_tk = x1(2); theta1_tk = x1(3); v1_tk = x1(4);
xU_tk = xU(1); yU_tk = xU(2); thetaU_tk = xU(3); vU_tk = xU(4);


%% safety constraints
% safety with truck    % xU,yU,theta_U,vU,xC,yC,theta_C,vC
L_fU = @(X) 2*(X(1)-X(5))/(aC^2)*(vU-X(8)*cos(X(7))) ...
    + 2*(X(2)-X(6))/(bC^2)*(-X(8)*sin(X(7)))...
    + kU*((X(1)-X(5))^2/(aC^2)+(X(2)-X(6))^2/(bC^2)-X(8)^2);    % L_fU+b_U
L_gU2 = @(X) 2*(X(1)-X(5))/(aC^2)*(X(8)*sin(X(7))) - 2*(X(2)-X(6))/(bC^2)*(X(8)*cos(X(7)));   % phi_C
L_gU3 = @(X) -2*X(8);  % uC
nonlcon_U = @ellipse_safety_U;
A_U = [];
b_U = [];
Aeq_U = [];
beq_U = [];
x0_U = [xU_tk, yU_tk, thetaU_tk, vU_tk, xC_tk, yC_tk, thetaC_tk, vC_tk];
lb_U = [xU_tk-s_xU, yU_tk-s_yU, thetaU_tk-s_thetaU, vU_tk-s_vU, xC_tk-s_xc, yC_tk-s_yc, thetaC_tk-s_thetaC, vC_tk-s_vc];
ub_U = [xU_tk+s_xU, yU_tk+s_yU, thetaU_tk+s_thetaU, vU_tk+s_vU, xC_tk+s_xc, yC_tk+s_yc, thetaC_tk+s_thetaC, vC_tk+s_vc];
options = optimoptions('fmincon','Display','off','Algorithm','sqp');
[X_LfU, minL_fU] = fmincon(L_fU,x0_U,A_U,b_U,Aeq_U,beq_U,lb_U,ub_U,nonlcon_U,options);
L_fbU = minL_fU;

if phi_C >= 0
    [X_LgU2, minL_gU2] = fmincon(L_gU2,x0_U,A_U,b_U,Aeq_U,beq_U,lb_U,ub_U,nonlcon_U,options);
else
    L_gU2 = @(X) -2*(X(1)-X(5))/(aC^2)*(X(8)*sin(X(7))) + 2*(X(2)-X(6))/(bC^2)*(X(8)*cos(X(7)));   % phi_C
    [X_LgU2, fval] = fmincon(L_gU2,x0_U,A_U,b_U,Aeq_U,beq_U,lb_U,ub_U,nonlcon_U,options);
    minL_gU2 = -fval;
end
L_gbU2 = minL_gU2;

if uC >= 0
    [X_LgU3, minL_gU3] = fmincon(L_gU3,x0_U,A_U,b_U,Aeq_U,beq_U,lb_U,ub_U,nonlcon_U,options);
else
    L_gU3 = @(X) 2*X(8);
    [X_LgU3, fval] = fmincon(L_gU3,x0_U,A_U,b_U,Aeq_U,beq_U,lb_U,ub_U,nonlcon_U,options);
    minL_gU3 = -fval;
end
L_gbU3 = minL_gU3;

LfBU = L_fbU;
LgBU = [L_gbU3,0,L_gbU2,0];     % U = [uC, u1, phi_C, phi_1]
classKU = 0;



%
% safety 1 & C
% x1,y1,theta_1,v1,xC,yC,theta_C,vC
L_f2 = @(X2) 2*(X2(1)-X2(5))/(aC^2)*(X2(4)*cos(X2(3))-X2(8)*cos(X2(7))) ...
    + 2*(X2(2)-X2(6))/(bC^2)*(X2(4)*sin(X2(3))-X2(8)*sin(X2(7)))...
    + k2*((X2(1)-X2(5))^2/(aC^2)+(X2(2)-X2(6))^2/(bC^2)-X2(8)^2);    % L_f2+b_2
L_g21 = @(X2) -2*(X2(1)-X2(5))/(aC^2)*(X2(4)*sin(X2(3))) + 2*(X2(2)-X2(6))/(bC^2)*(X2(4)*cos(X2(3)));  % phi_1
L_g22 = @(X2) 2*(X2(1)-X2(5))/(aC^2)*(X2(8)*sin(X2(7))) - 2*(X2(2)-X2(6))/(bC^2)*(X2(8)*cos(X2(7)));   % phi_C
L_g23 = @(X2) -2*X2(4);  % u1
nonlcon_2 = @ellipse_safety_2;
A_2 = [];
b_2 = [];
Aeq_2 = [];
beq_2 = [];
x0_2 = [x1_tk, y1_tk, theta1_tk, v1_tk, xC_tk, yC_tk, thetaC_tk, vC_tk];
lb_2 = [x1_tk-s_x1, y1_tk-s_y1, theta1_tk-s_theta1, v1_tk-s_v1, xC_tk-s_xc, yC_tk-s_yc, thetaC_tk-s_thetaC, vC_tk-s_vc];
ub_2 = [x1_tk+s_x1, y1_tk+s_y1, theta1_tk+s_theta1, v1_tk+s_v1, xC_tk+s_xc, yC_tk+s_yc, thetaC_tk+s_thetaC, vC_tk+s_vc];
[X_Lf2, minL_f2] = fmincon(L_f2,x0_2,A_2,b_2,Aeq_2,beq_2,lb_2,ub_2,nonlcon_2,options);
L_fb2 = minL_f2;

if phi_1 >= 0
    [X_Lg21, minL_g21] = fmincon(L_g21,x0_2,A_2,b_2,Aeq_2,beq_2,lb_2,ub_2,nonlcon_2,options);
else
    L_g21 = @(X2) 2*(X2(1)-X2(5))/(aC^2)*(X2(4)*sin(X2(3))) - 2*(X2(2)-X2(6))/(bC^2)*(X2(4)*cos(X2(3)));  % phi_1
    [X_Lg21, fval] = fmincon(L_g21,x0_2,A_2,b_2,Aeq_2,beq_2,lb_2,ub_2,nonlcon_2,options);
    minL_g21 = -fval;
end
L_gb21 = minL_g21;

if phi_C >= 0
    [X_Lg22, minL_g22] = fmincon(L_g22,x0_2,A_2,b_2,Aeq_2,beq_2,lb_2,ub_2,nonlcon_2,options);
else
    L_g22 = @(X2) -2*(X2(1)-X2(5))/(aC^2)*(X2(8)*sin(X2(7))) + 2*(X2(2)-X2(6))/(bC^2)*(X2(8)*cos(X2(7)));   % phi_C
    [X_Lg22, fval] = fmincon(L_g22,x0_2,A_2,b_2,Aeq_2,beq_2,lb_2,ub_2,nonlcon_2,options);
    minL_g22 = -fval;
end
L_gb22 = minL_g22;

if u1 >= 0
    [X_Lg23, minL_g23] = fmincon(L_g23,x0_2,A_2,b_2,Aeq_2,beq_2,lb_2,ub_2,nonlcon_2,options);
else
    L_g23 = @(X2) 2*X2(4);
    [X_Lg23, fval] = fmincon(L_g23,x0_2,A_2,b_2,Aeq_2,beq_2,lb_2,ub_2,nonlcon_2,options);
    minL_g23 = -fval;
end
L_gb23 = minL_g23;

LfB2 = L_fb2;
LgB2 = [0,L_gb23,L_gb22,L_gb21];  
classK2 = 0;

%

%     % state limits
LfB4 = xC(4)*sin(xC(3));
LgB4 = [0,0,xC(4)*cos(xC(3)),0];
classK4 = k4*(xC(2)+lanewidth/2-1);

LfB5 = -xC(4)*sin(xC(3));
LgB5 = [0,0,-xC(4)*cos(xC(3)),0];
classK5 = k5*(3/2*lanewidth-1-xC(2));

LfB6 = x1(4)*sin(x1(3));
LgB6 = [0,0,0,x1(4)*cos(x1(3))];
classK6 = k6*(x1(2)-lanewidth/2-1);

LfB7 = -x1(4)*sin(x1(3));
LgB7 = [0,0,0,-x1(4)*cos(x1(3))];
classK7 = k7*(3/2*lanewidth-1-x1(2));


LfB10 = 0;
LgB10 = [0,1,0,0];
classK10 = k10*(x1(4)-v_min);

LfB11 = 0;
LgB11 = [0,-1,0,0];
classK11 = k11*(v_max-x1(4));

LfB12 = 0;
LgB12 = [1,0,0,0];
classK12 = k12*(xC(4)-v_min);

LfB13 = 0;
LgB13 = [-1,0,0,0];
classK13 = k13*(v_max-xC(4));


% CLFs
LfV1 = 0;
LgV1 = [2*(xC(4)-v_des),0,0,0];
classV1 = vareps1*(xC(4)-v_des)^2;

LfV2 = 0;
LgV2 = [0,2*(x1(4)-v_des),0,0];
classV2 = vareps2*(x1(4)-v_des)^2;

LfV3 = 2*(xC(2)-lanewidth)*xC(4)*sin(xC(3));
LgV3 = [0,0,2*(xC(2)-lanewidth)*xC(4)*cos(xC(3)),0];
classV3 = 100*vareps3*(xC(2)-lanewidth)^2;        % lane changing of C


LfV6 = 2*(x1(2)-lanewidth)*x1(4)*sin(x1(3));
LgV6 = [0,0,0,2*(x1(2)-lanewidth)*x1(4)*cos(x1(3))];
classV6 = vareps6*(x1(2)-lanewidth)^2;        % lane keeping of 1

%% Solve QP

H = eye(8);

F = [0;0;0;0;0;0;0;0];
A = [
    -LgBU,0,0,0,0;
    -LgB2,0,0,0,0;
    -LgB4,0,0,0,0;
    -LgB5,0,0,0,0;
    -LgB6,0,0,0,0;
    -LgB7,0,0,0,0;
    -LgB10,0,0,0,0;
    -LgB11,0,0,0,0;
    -LgB12,0,0,0,0;
    -LgB13,0,0,0,0;
    LgV1,-1,0,0,0;
    LgV2,0,-1,0,0;
    LgV3,0,0,-1,0;
    LgV6,0,0,0,-1
    ];
b = [
    LfBU+classKU;
    LfB2+classK2;
    LfB4+classK4;
    LfB5+classK5;
    LfB6+classK6;
    LfB7+classK7;
    LfB10+classK10;
    LfB11+classK11;
    LfB12+classK12;
    LfB13+classK13;
    -LfV1-classV1;
    -LfV2-classV2;
    -LfV3-classV3;
    -LfV6-classV6
    ];
lb = [-7,-7,-pi/4,-pi/4,0,0,0,0];
ub = [3.3, 3.3, pi/4, pi/4, Inf, Inf, Inf, Inf];
U0 = [uC,u1,phi_C,phi_1,0,0,0,0];
options = optimoptions('quadprog','Algorithm','active-set','Display','off');
[U,fval,exitflag,output,labmda] = quadprog(H,F,A,b,[],[],lb,ub,U0,options);
if numel(U) == 0
    return
else
    uC = U(1);
    u1 = U(2);
    phi_C = U(3);
    phi_1 = U(4);

end

%% update dynamics
    xU = [xU(1)+vU*dt, 0, 0, 20];
    
    t = [0 dt];
    [~,statesC] = ode45(@C_dynamics,t,[xC,uC,phi_C]);
    xC = statesC(end,1:4);
    

    [~,states1] = ode45(@frontCAV_dynamics,t,[x1,u1,phi_1]);
    x1 = states1(end,1:4);
  

    vector = [xU,xC,x1,uC,u1,phi_1,phi_C,xC_tk,yC_tk,vC_tk,x1_tk,y1_tk,v1_tk,...
            xU_tk,yU_tk,vU_tk];
    




%% helpful functions


    function [c2,ceq2] = ellipse_safety_2(X2)
        aC = 0.6; bC = 0.1;
        c2 = X2(8)^2 - (X2(1)-X2(5))^2/(aC^2) - (X2(2)-X2(6))^2/(bC^2);
        ceq2 = [];
    end



    function [cU,ceqU] = ellipse_safety_U(X)
        aC = 0.6; bC = 0.1;
        cU = X(8)^2 - (X(1)-X(5))^2/(aC^2) - (X(2)-X(6))^2/(bC^2);
        ceqU = [];
    end

end
