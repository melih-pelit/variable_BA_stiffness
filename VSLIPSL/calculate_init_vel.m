function dq0 = calculate_init_vel(X, param, dx_CoM_des, dz_CoM_des, des_x_sw, des_z_sw)
% function for calculating initial velocity of bipedal robot

%% positions
th1 = X(1);
th2 = X(2);
th3 = X(3);
th4 = X(4);
th5 = X(5);

th = [th1; th2; th3; th4; th5];
%% parameters

m1 = param(1);
m2 = param(2);
m3 = m2;
m4 = m1;
m5 = param(3);

l1 = param(4);
l2 = param(5);
l5 = param(6);
l3 = l2;
l4 = l1;

gravi = param(7);
I1 = param(8);
I2 = param(9);
I5 = param(10);
I3 = I1;
I4 = I2;

x1 = 0;
z1 = 0;

param = [m1; m2; m5; l1; l2; l5; gravi; I1; I2; I5];
%% fmincon
% des_x_sw = 2.5; % desired x velocity of swing foot
% des_z_sw = 0.1; % desired z velocity of swing foot

% dx_sw and dz_sw constraints and body velocity (dth1 + dth2 + dth5)
fun = @(x)(-l1*x(1)*sin(th1) - l2*(x(1) + x(2))*sin(th1 + th2) -l3*(x(1) + x(2) + x(3))*sin(th1 + th2 + th3) - l4*(x(1) + x(2) + x(3) + x(4))*sin(th1 + th2 + th3 + th4) - des_x_sw)^2 + (l1*x(1)*cos(th1) + l2*(x(1) + x(2))*cos(th1 + th2) + l3*(x(1) + x(2) + x(3))*cos(th1 + th2 + th3) + l4*(x(1) + x(2) + x(3) + x(4))*cos(th1 + th2 + th3 + th4) - des_z_sw)^2 + (x(1) + x(2) + x(5))^2;
x0 = [0; 0; 0; 0; 0];
% I didn't put the A*x<b limits for now
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
nonlcon = @(x)init_vel_const(x,param, th, dx_CoM_des, dz_CoM_des);
x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon);
dq0 = x;
end

function [c,ceq] = init_vel_const(x,param, th, dx_CoM_des, dz_CoM_des)


%% positions
th1 = th(1);
th2 = th(2);
th3 = th(3);
th4 = th(4);
th5 = th(5);

%% velocities
dth1 = x(1);
dth2 = x(2);
dth3 = x(3);
dth4 = x(4);
dth5 = x(5);

vars = [th; x];

%% parameters

m1 = param(1);
m2 = param(2);
m3 = m2;
m4 = m1;
m5 = param(3);
m_t = m1 + m2 + m3 + m4 + m5;

l1 = param(4);
l2 = param(5);
l5 = param(6);
l3 = l2;
l4 = l1;

gravi = param(7);
I1 = param(8);
I2 = param(9);
I5 = param(10);
I3 = I1;
I4 = I2;

%% parameters

E_des = 824; % [N/m]

%% Calculation of CoM velocity

% dx_G: CoM velocities in x direction
dx_G = zeros(5);
dx_G(1) = -(l1/2)*dth1*sin(th1);
dx_G(2) = -l1*dth1*sin(th1) - (l2/2)*(dth1 + dth2)*sin(th1 + th2);
dx_G(3) = -l1*dth1*sin(th1) - l2*(dth1 + dth2)*sin(th1 + th2) - (l3/2)*(dth1 + dth2 + dth3)*sin(th1 + th2 + th3);
dx_G(4) = -l1*dth1*sin(th1) - l2*(dth1 + dth2)*sin(th1 + th2) - l3*(dth1 + dth2 + dth3)*sin(th1 + th2 + th3) - (l4/2)*(dth1 + dth2 + dth3 + dth4)*sin(th1 + th2 + th3 + th4);
dx_G(5) = -l1*dth1*sin(th1) - l2*(dth1 + dth2)*sin(th1 + th2) - (l5/2)*(dth1 + dth2 + dth5)*sin(th1 + th2 + th5);

% dy_g: CoM velocities in y direction
dz_G = zeros(5);
dz_G(1) = (l1/2)*dth1*cos(th1);
dz_G(2) = l1*dth1*cos(th1) + (l2/2)*(dth1 + dth2)*cos(th1 + th2);
dz_G(3) = l1*dth1*cos(th1) + l2*(dth1 + dth2)*cos(th1 + th2) + (l3/2)*(dth1 + dth2 + dth3)*cos(th1 + th2 + th3);
dz_G(4) = l1*dth1*cos(th1) + l2*(dth1 + dth2)*cos(th1 + th2) + l3*(dth1 + dth2 + dth3)*cos(th1 + th2 + th3) + (l4/2)*(dth1 + dth2 + dth3 + dth4)*cos(th1 + th2 + th3 + th4);
dz_G(5) = l1*dth1*cos(th1) + l2*(dth1 + dth2)*cos(th1 + th2) + (l5/2)*(dth1 + dth2 + dth5)*cos(th1 + th2 + th5);

dxG = (m1*dx_G(1) + m2*dx_G(2) + m3*dx_G(3) + m4*dx_G(4) + m5*dx_G(5))/(m1 + m2 + m3 + m4 + m5);
dzG = (m1*dz_G(1) + m2*dz_G(2) + m3*dz_G(3) + m4*dz_G(4) + m5*dz_G(5))/(m1 + m2 + m3 + m4 + m5);

%% desired values
% for q1_0 = 0; q2_0 = 0.979061037172950; theta_p0 = -0.00258987145223157; % initial dq2_0 / dq1_0
% dx_CoM_des = 1.24067772387984;
% dz_CoM_des = 0.483515704137646;

%% constraint functions
c = [];
% ceq = [dx_CoM, dz_CoM, E_total]
% ceq = [ dxG - dx_CoM_des;
%         dzG - dz_CoM_des;
%         calculate_total_energy(vars, param) - E_des];

ceq = [ dxG - dx_CoM_des;
        dzG - dz_CoM_des];
end
