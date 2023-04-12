function [q0, dq0] = calculate_init_pos(x_CoM_des, z_CoM_des, x_sw_des, z_sw_des, dx_CoM_des, dz_CoM_des, des_x_sw, des_z_sw, param)
% v1 - solving init conds with solve()
% v2 - solving init conds with fmincon()
% 09.08.2018 - modified for fully actuated model


%% Parameters

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

r_k = param(11);
r_h = param(12);
k_ba = param(13);
phi_h0 = param(14);
phi_k0 = param(15);

x1 = 0;
z1 = 0;
%% fmincon
% x_sw and z_sw constraints
% x_sw_des = -0.5333;
% z_sw_des = 0.001; % desired height of the swing foot @ init
fun = @(x)(l1*cos(x(1)) + l2*cos(x(1) + x(2)) + l3*cos(x(1) + x(2) + x(3)) + l4*cos(x(1) + x(2) + x(3) + x(4)) - x_sw_des)^2 + 10*(l1*sin(x(1)) + l2*sin(x(1) + x(2)) + l3*sin(x(1) + x(2) + x(3)) + l4*sin(x(1) + x(2) + x(3) + x(4)) - z_sw_des)^2;
x0 = [0; 0; 0; 0; 0;];
A = [ 1  0  0 0 0;
     -1  0  0 0 0;
      1  1  0 0 0;
     -1 -1  0 0 0;
      1  1  1 0 0;
     -1 -1 -1 0 0];
b = [pi/2; 0; pi; 0; 2*pi; -pi*1.5];
Aeq = [];
beq = [];
lb = [];
ub = [];
nonlcon = @(x)init_pos_const(x,param, x_CoM_des, z_CoM_des);
x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon);
q0 = x;
dq0 = calculate_init_vel(q0, param, dx_CoM_des, dz_CoM_des, des_x_sw, des_z_sw);
draw_robot(x, param)
end

function [c,ceq] = init_pos_const(x,param, x_CoM_des, z_CoM_des)

th1 = x(1);
th2 = x(2);
th3 = x(3);
th4 = x(4);
th5 = x(5);

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

%% 
% x CoM's of respective links
x1G = x1 + (l1/2)*cos(th1); 
x2G = x1 + l1*cos(th1) + (l2/2)*cos(th1+th2);
x3G = x1 + l1*cos(th1) + l2*cos(th1+th2) + (l3/2)*cos(th1 + th2 + th3);
x4G = x1 + l1*cos(th1) + l2*cos(th1+th2) + l3*cos(th1 + th2 + th3) + (l4/2)*cos(th1 + th2 + th3 + th4);
xbG = x1 + l1*cos(th1) + l2*cos(th1+th2) + (l5/2)*cos(th1 + th2 + th5);

% y CoM's of respective links
z1G = z1 + (l1/2)*sin(th1);
z2G = z1 + l1*sin(th1) + (l2/2)*sin(th1+th2);
z3G = z1 + l1*sin(th1) + l2*sin(th1+th2) + (l3/2)*sin(th1 + th2 + th3);
z4G = z1 + l1*sin(th1) + l2*sin(th1+th2) + l3*sin(th1 + th2 + th3) + (l4/2)*sin(th1 + th2 + th3 + th4);
zbG = z1 + l1*sin(th1) + l2*sin(th1+th2) + (l5/2)*sin(th1 + th2 + th5);

% c = [th1 + th2 + th5 - pi/2 - deg2rad(5);
%     -(th1 + th2 + th5 - pi/2) - deg2rad(0)];

c = [];
% ceq = [x_CoM, z_CoM, thb];
% x_CoM_des = -0.170565213819003; % values got from slipTraj_201912231604 lift off
% z_CoM_des = 0.932190116079923;  % values got from slipTraj_201912231604 lift off
% ceq = [(m1*x1G + m2*x2G + m3*x3G + m4*x4G + m5*xbG)/(m1 + m2 + m3 + m4 + m5) - x_CoM_des;
%     (m1*z1G + m2*z2G + m3*z3G + m4*z4G + m5*zbG)/(m1 + m2 + m3 + m4 + m5) - z_CoM_des];

% ceq = [(m1*x1G + m2*x2G + m3*x3G + m4*x4G + m5*xbG)/(m1 + m2 + m3 + m4 + m5) - x_CoM_des;
%     (m1*z1G + m2*z2G + m3*z3G + m4*z4G + m5*zbG)/(m1 + m2 + m3 + m4 + m5) - z_CoM_des;
%     th1 + th2 + th5 - pi/2 + deg2rad(15)];

ceq = [(m1*x1G + m2*x2G + m3*x3G + m4*x4G + m5*xbG)/(m1 + m2 + m3 + m4 + m5) - x_CoM_des;
    (m1*z1G + m2*z2G + m3*z3G + m4*z4G + m5*zbG)/(m1 + m2 + m3 + m4 + m5) - z_CoM_des;
    th1 + th2 + th5 - pi/2 + deg2rad(0)];

end