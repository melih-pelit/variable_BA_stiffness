% code for deriving the Jacobian 20.08.2017, notebook
clear all
clc
%% variables of the model
syms th1 th2 th3 th4 th5 x1 z1
syms dth1 dth2 dth3 dth4 dth5
syms ddth1 ddth2 ddth3 ddth4 ddth5
syms m1 m2 m3 m4 m5
syms I1 I2 I3 I4 I5
syms l1 l2 l3 l4 l5
syms gravi x1

%% CoM locations

% x_g CoM's of respective links
x_G = sym(zeros(5));
x_G(1) = x1 + (l1/2)*cos(th1); 
x_G(2) = x1 + l1*cos(th1) + (l1/2)*cos(th1 + th2);
x_G(3) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + (l3/2)*cos(th1 + th2 + th3);
x_G(4) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + (l4/2)*cos(th1 + th2 + th3 + th4);
x_G(5) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + (l5/2)*cos(th1 + th2 + th5); % body

% z_g CoM's of respective links
z_G = sym(zeros(5));
z_G(1) = (l1/2)*sin(th1); 
z_G(2) = l1*sin(th1) + (l2/2)*sin(th1 + th2);
z_G(3) = l1*sin(th1) + l2*sin(th1 + th2) + (l3/2)*sin(th1 + th2 + th3);
z_G(4) = l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + (l4/2)*sin(th1 + th2 + th3 + th4);
z_G(5) = l1*sin(th1) + l2*sin(th1 + th2) + (l5/2)*sin(th1 + th2 + th5);

xG = (m1*x_G(1) + m2*x_G(2) + m3*x_G(3) + m4*x_G(4) + m5*x_G(5))/(m1 + m2 + m3 + m4 + m5);
zG = (m1*z_G(1) + m2*z_G(2) + m3*z_G(3) + m4*z_G(4) + m5*z_G(5))/(m1 + m2 + m3 + m4 + m5);

q = [th1; th2; th3; th4; th5];
dq = [dth1; dth2; dth3; dth4; dth5];
ddq = [ddth1; ddth2; ddth3; ddth4; ddth5];

%% Swing foot locations

x_F = x1 + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + l4*cos(th1 + th2 + th3 + th4);
z_F = l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + l4*sin(th1 + th2 + th3 + th4);

%% Deriving the Jacobian for single stance

x_t = [xG; zG; x_F; z_F; th5];
J_t = jacobian(x_t, q);

for i=1:1:length(q)
    for j= 1:1:length(q)
        fprintf('J_t%d%d', i ,j)
        J_t(i,j)
    end
end

%% Deriving time derivative of the Jacobian for singe stance

dJ_t = sym(zeros(5, 5));
jacobSize = size(J_t);
for i = 1:1:jacobSize(1)
    for j = 1:1:jacobSize(2)
        dJ_t(i,j) = simplify(jacobian(J_t(i, j), q) * dq);
    end
end

for i=1:1:length(q)
    for j= 1:1:length(q)
        fprintf('dJ_t%d%d', i ,j)
        dJ_t(i,j)
    end
end

%% Deriving the Jacobian for double stance

x_t_db = [xG; zG; th5]; % task space for double stance phase
J_t_db = jacobian(x_t_db, q);
jacobDbSize = size(J_t_db);
for i=1:1:jacobDbSize(1)
    for j= 1:1:jacobDbSize(2)
        fprintf('J_t_DS%d%d', i ,j)
        J_t_db(i,j)
    end
end
%% Deriving time derivative of the Jacobian for double stance

% dJ_t_db = sym(zeros(5, 5));

for i = 1:1:jacobDbSize(1)
    for j = 1:1:jacobDbSize(2)
        dJ_t_db(i,j) = simplify(jacobian(J_t_db(i, j), q) * dq);
    end
end

for i=1:1:jacobDbSize(1)
    for j= 1:1:jacobDbSize(2)
        fprintf('dJ_t_DS%d%d', i ,j)
        dJ_t_db(i,j)
    end
end

%% deriving J_hip_sw and J_hip_st, jacobians mapping swing and stance leg
% velocities to hip velocity

% Hip position using stance foot angles
% th2_abs = th1 + th2
syms th1_abs th2_abs th3_abs th4_abs
syms dth1_abs dth2_abs dth3_abs dth4_abs
q_st = [th1_abs; th2_abs];
dq_st = [dth1_abs; dth2_abs];
x_hip_st = l1*cos(th1_abs) + l2*cos(th2_abs);
z_hip_st = l1*sin(th1_abs) + l2*sin(th2_abs);

J_hip_st = jacobian([x_hip_st; z_hip_st], q_st);

% Hip position using sw foot(soon to be) angles
% th3_abs = th1 + th2 + th3 & th4_abs = th1 + th2 + th3 + th4
q_sw = [th3_abs; th4_abs];
dq_sw = [dth3_abs; dth4_abs];
x_hip_sw = l4 * cos(th4_abs) + l3*cos(th3_abs);
z_hip_sw = l4 * sin(th4_abs) + l3*sin(th3_abs);

J_hip_sw = jacobian([x_hip_sw; z_hip_sw], q_sw);

Gamma_st = [inv(J_hip_st)*J_hip_sw, zeros(2,1); eye(3,3)];
Gamma_sw = [inv(J_hip_sw)*J_hip_st, zeros(2,1); eye(3,3)];

Gamma_st_Size = size(Gamma_st);
for i = 1:1:Gamma_st_Size(1)
    for j = 1:1:Gamma_st_Size(2)
        dGamma_st(i,j) = simplify(jacobian(Gamma_st(i, j), q_st) * dq_st);
    end
end

Gamma_sw_Size = size(Gamma_sw);
for i = 1:1:Gamma_sw_Size(1)
    for j = 1:1:Gamma_sw_Size(2)
        dGamma_sw(i,j) = simplify(jacobian(Gamma_sw(i, j), q_sw) * dq_sw);
    end
end

%% Calculating the accelering of CoM of 5 Link robot
dxG = jacobian(xG, q)*dq;
dzG = jacobian(zG, q)*dq;

ddxG = jacobian(dxG, q)*dq + jacobian(dxG, dq)*ddq;
ddzG = jacobian(dzG, q)*dq + jacobian(dzG, dq)*ddq;

%% calculating acceleration of the swing foot (swing foot in the double stance phase)
% dq_sw_ft = 0; -> J_c * dq = 0;

%% Calculating the disturbance jacobian for the hip

x_hip = l1*cos(th1) + l2*cos(th1 + th2);
y_hip = l1*sin(th1) + l2*sin(th1 + th2);

J_dist = jacobian([x_hip; y_hip], q);