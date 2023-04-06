function  [ddq_GRF, GRF] = EoM_floating(t, X, param, flag, U)
% 2020.10.22 function to calculate the ground reaction forces in the
% actuated ankle using the floating model with q = [x_st; y_st; th1;th2;th3;th4;th5]

%% variables
th1 = X(1);
th2 = X(2);
th3 = X(3);
th4 = X(4);
th5 = X(5);

x_st = flag(2); % x position of the stance foot
y_st = 0; % y position of the stance foot

dth1 = X(6);
dth2 = X(7);
dth3 = X(8);
dth4 = X(9);
dth5 = X(10);

dx_st = 0; % setting x velocity of the stance foot to zero because it is stationary
dy_st = 0;

q = [x_st; y_st; th1;th2;th3;th4;th5];
dq = [dx_st; dy_st; dth1;dth2;dth3;dth4;dth5];

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

r_k = param(11);
r_h = param(12);
k_ba = param(13);
phi_h0 = param(14);
phi_k0 = param(15);

%% flags
f_SS = flag(1); % indicates which stance phase it is
foot = flag(2); % location of the stance foot
foot_prev = flag(3); % previous location of the stance foot

%% Inertia Matrix
MM = MM_matrix_relative_floating(X, param, flag);

%% Centrifugal force · Coriolis force Vector
CC = zeros(5,1);

CC(1, 1) = - (dth1^2*l3*m3*cos(th1 + th2 + th3))/2 - dth1^2*l3*m4*cos(th1 + th2 + th3) - (dth2^2*l3*m3*cos(th1 + th2 + th3))/2 - dth2^2*l3*m4*cos(th1 + th2 + th3) - (dth3^2*l3*m3*cos(th1 + th2 + th3))/2 - dth3^2*l3*m4*cos(th1 + th2 + th3) - (dth1^2*l5*m5*cos(th1 + th2 + th5))/2 - (dth2^2*l5*m5*cos(th1 + th2 + th5))/2 - (dth5^2*l5*m5*cos(th1 + th2 + th5))/2 - (dth1^2*l4*m4*cos(th1 + th2 + th3 + th4))/2 - (dth2^2*l4*m4*cos(th1 + th2 + th3 + th4))/2 - (dth3^2*l4*m4*cos(th1 + th2 + th3 + th4))/2 - (dth4^2*l4*m4*cos(th1 + th2 + th3 + th4))/2 - (dth1^2*l2*m2*cos(th1 + th2))/2 - dth1^2*l2*m3*cos(th1 + th2) - (dth2^2*l2*m2*cos(th1 + th2))/2 - dth1^2*l2*m4*cos(th1 + th2) - dth2^2*l2*m3*cos(th1 + th2) - dth1^2*l2*m5*cos(th1 + th2) - dth2^2*l2*m4*cos(th1 + th2) - dth2^2*l2*m5*cos(th1 + th2) - (dth1^2*l1*m1*cos(th1))/2 - dth1^2*l1*m2*cos(th1) - dth1^2*l1*m3*cos(th1) - dth1^2*l1*m4*cos(th1) - dth1^2*l1*m5*cos(th1) - dth1*dth2*l3*m3*cos(th1 + th2 + th3) - 2*dth1*dth2*l3*m4*cos(th1 + th2 + th3) - dth1*dth3*l3*m3*cos(th1 + th2 + th3) - 2*dth1*dth3*l3*m4*cos(th1 + th2 + th3) - dth2*dth3*l3*m3*cos(th1 + th2 + th3) - 2*dth2*dth3*l3*m4*cos(th1 + th2 + th3) - dth1*dth2*l5*m5*cos(th1 + th2 + th5) - dth1*dth5*l5*m5*cos(th1 + th2 + th5) - dth2*dth5*l5*m5*cos(th1 + th2 + th5) - dth1*dth2*l4*m4*cos(th1 + th2 + th3 + th4) - dth1*dth3*l4*m4*cos(th1 + th2 + th3 + th4) - dth1*dth4*l4*m4*cos(th1 + th2 + th3 + th4) - dth2*dth3*l4*m4*cos(th1 + th2 + th3 + th4) - dth2*dth4*l4*m4*cos(th1 + th2 + th3 + th4) - dth3*dth4*l4*m4*cos(th1 + th2 + th3 + th4) - dth1*dth2*l2*m2*cos(th1 + th2) - 2*dth1*dth2*l2*m3*cos(th1 + th2) - 2*dth1*dth2*l2*m4*cos(th1 + th2) - 2*dth1*dth2*l2*m5*cos(th1 + th2);
CC(2, 1) = - (dth1^2*l3*m3*sin(th1 + th2 + th3))/2 - dth1^2*l3*m4*sin(th1 + th2 + th3) - (dth2^2*l3*m3*sin(th1 + th2 + th3))/2 - dth2^2*l3*m4*sin(th1 + th2 + th3) - (dth3^2*l3*m3*sin(th1 + th2 + th3))/2 - dth3^2*l3*m4*sin(th1 + th2 + th3) - (dth1^2*l5*m5*sin(th1 + th2 + th5))/2 - (dth2^2*l5*m5*sin(th1 + th2 + th5))/2 - (dth5^2*l5*m5*sin(th1 + th2 + th5))/2 - (dth1^2*l4*m4*sin(th1 + th2 + th3 + th4))/2 - (dth2^2*l4*m4*sin(th1 + th2 + th3 + th4))/2 - (dth3^2*l4*m4*sin(th1 + th2 + th3 + th4))/2 - (dth4^2*l4*m4*sin(th1 + th2 + th3 + th4))/2 - (dth1^2*l2*m2*sin(th1 + th2))/2 - dth1^2*l2*m3*sin(th1 + th2) - (dth2^2*l2*m2*sin(th1 + th2))/2 - dth1^2*l2*m4*sin(th1 + th2) - dth2^2*l2*m3*sin(th1 + th2) - dth1^2*l2*m5*sin(th1 + th2) - dth2^2*l2*m4*sin(th1 + th2) - dth2^2*l2*m5*sin(th1 + th2) - (dth1^2*l1*m1*sin(th1))/2 - dth1^2*l1*m2*sin(th1) - dth1^2*l1*m3*sin(th1) - dth1^2*l1*m4*sin(th1) - dth1^2*l1*m5*sin(th1) - dth1*dth2*l3*m3*sin(th1 + th2 + th3) - 2*dth1*dth2*l3*m4*sin(th1 + th2 + th3) - dth1*dth3*l3*m3*sin(th1 + th2 + th3) - 2*dth1*dth3*l3*m4*sin(th1 + th2 + th3) - dth2*dth3*l3*m3*sin(th1 + th2 + th3) - 2*dth2*dth3*l3*m4*sin(th1 + th2 + th3) - dth1*dth2*l5*m5*sin(th1 + th2 + th5) - dth1*dth5*l5*m5*sin(th1 + th2 + th5) - dth2*dth5*l5*m5*sin(th1 + th2 + th5) - dth1*dth2*l4*m4*sin(th1 + th2 + th3 + th4) - dth1*dth3*l4*m4*sin(th1 + th2 + th3 + th4) - dth1*dth4*l4*m4*sin(th1 + th2 + th3 + th4) - dth2*dth3*l4*m4*sin(th1 + th2 + th3 + th4) - dth2*dth4*l4*m4*sin(th1 + th2 + th3 + th4) - dth3*dth4*l4*m4*sin(th1 + th2 + th3 + th4) - dth1*dth2*l2*m2*sin(th1 + th2) - 2*dth1*dth2*l2*m3*sin(th1 + th2) - 2*dth1*dth2*l2*m4*sin(th1 + th2) - 2*dth1*dth2*l2*m5*sin(th1 + th2);
CC(3, 1) = - (dth2^2*l1*l3*m3*sin(th2 + th3))/2 - dth2^2*l1*l3*m4*sin(th2 + th3) - (dth3^2*l1*l3*m3*sin(th2 + th3))/2 - dth3^2*l1*l3*m4*sin(th2 + th3) - (dth2^2*l1*l5*m5*sin(th2 + th5))/2 - (dth3^2*l2*l4*m4*sin(th3 + th4))/2 - (dth4^2*l2*l4*m4*sin(th3 + th4))/2 - (dth5^2*l1*l5*m5*sin(th2 + th5))/2 - (dth2^2*l1*l2*m2*sin(th2))/2 - dth2^2*l1*l2*m3*sin(th2) - dth2^2*l1*l2*m4*sin(th2) - dth2^2*l1*l2*m5*sin(th2) - (dth3^2*l2*l3*m3*sin(th3))/2 - dth3^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 - (dth5^2*l2*l5*m5*sin(th5))/2 - (dth2^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - (dth3^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - (dth4^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - dth1*dth2*l1*l3*m3*sin(th2 + th3) - 2*dth1*dth2*l1*l3*m4*sin(th2 + th3) - dth1*dth3*l1*l3*m3*sin(th2 + th3) - 2*dth1*dth3*l1*l3*m4*sin(th2 + th3) - dth2*dth3*l1*l3*m3*sin(th2 + th3) - 2*dth2*dth3*l1*l3*m4*sin(th2 + th3) - dth1*dth2*l1*l5*m5*sin(th2 + th5) - dth1*dth3*l2*l4*m4*sin(th3 + th4) - dth1*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth3*l2*l4*m4*sin(th3 + th4) - dth2*dth4*l2*l4*m4*sin(th3 + th4) - dth1*dth5*l1*l5*m5*sin(th2 + th5) - dth3*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth5*l1*l5*m5*sin(th2 + th5) - dth1*dth2*l1*l2*m2*sin(th2) - 2*dth1*dth2*l1*l2*m3*sin(th2) - 2*dth1*dth2*l1*l2*m4*sin(th2) - 2*dth1*dth2*l1*l2*m5*sin(th2) - dth1*dth3*l2*l3*m3*sin(th3) - 2*dth1*dth3*l2*l3*m4*sin(th3) - dth2*dth3*l2*l3*m3*sin(th3) - 2*dth2*dth3*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4) - dth1*dth5*l2*l5*m5*sin(th5) - dth2*dth5*l2*l5*m5*sin(th5) - dth1*dth2*l1*l4*m4*sin(th2 + th3 + th4) - dth1*dth3*l1*l4*m4*sin(th2 + th3 + th4) - dth1*dth4*l1*l4*m4*sin(th2 + th3 + th4) - dth2*dth3*l1*l4*m4*sin(th2 + th3 + th4) - dth2*dth4*l1*l4*m4*sin(th2 + th3 + th4) - dth3*dth4*l1*l4*m4*sin(th2 + th3 + th4);
CC(4, 1) = (dth1^2*l1*l3*m3*sin(th2 + th3))/2 + dth1^2*l1*l3*m4*sin(th2 + th3) + (dth1^2*l1*l5*m5*sin(th2 + th5))/2 - (dth3^2*l2*l4*m4*sin(th3 + th4))/2 - (dth4^2*l2*l4*m4*sin(th3 + th4))/2 + (dth1^2*l1*l2*m2*sin(th2))/2 + dth1^2*l1*l2*m3*sin(th2) + dth1^2*l1*l2*m4*sin(th2) + dth1^2*l1*l2*m5*sin(th2) - (dth3^2*l2*l3*m3*sin(th3))/2 - dth3^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 - (dth5^2*l2*l5*m5*sin(th5))/2 + (dth1^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - dth1*dth3*l2*l4*m4*sin(th3 + th4) - dth1*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth3*l2*l4*m4*sin(th3 + th4) - dth2*dth4*l2*l4*m4*sin(th3 + th4) - dth3*dth4*l2*l4*m4*sin(th3 + th4) - dth1*dth3*l2*l3*m3*sin(th3) - 2*dth1*dth3*l2*l3*m4*sin(th3) - dth2*dth3*l2*l3*m3*sin(th3) - 2*dth2*dth3*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4) - dth1*dth5*l2*l5*m5*sin(th5) - dth2*dth5*l2*l5*m5*sin(th5);
CC(5, 1) = (dth1^2*l1*l3*m3*sin(th2 + th3))/2 + dth1^2*l1*l3*m4*sin(th2 + th3) + (dth1^2*l2*l4*m4*sin(th3 + th4))/2 + (dth2^2*l2*l4*m4*sin(th3 + th4))/2 + (dth1^2*l2*l3*m3*sin(th3))/2 + dth1^2*l2*l3*m4*sin(th3) + (dth2^2*l2*l3*m3*sin(th3))/2 + dth2^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 + (dth1^2*l1*l4*m4*sin(th2 + th3 + th4))/2 + dth1*dth2*l2*l4*m4*sin(th3 + th4) + dth1*dth2*l2*l3*m3*sin(th3) + 2*dth1*dth2*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4);
CC(6, 1) = (l4*m4*(dth1^2*l2*sin(th3 + th4) + dth2^2*l2*sin(th3 + th4) + dth1^2*l3*sin(th4) + dth2^2*l3*sin(th4) + dth3^2*l3*sin(th4) + dth1^2*l1*sin(th2 + th3 + th4) + 2*dth1*dth2*l2*sin(th3 + th4) + 2*dth1*dth2*l3*sin(th4) + 2*dth1*dth3*l3*sin(th4) + 2*dth2*dth3*l3*sin(th4)))/2;
CC(7, 1) = (l5*m5*(dth1^2*l1*sin(th2 + th5) + dth1^2*l2*sin(th5) + dth2^2*l2*sin(th5) + 2*dth1*dth2*l2*sin(th5)))/2;

%% Gravity Vector
GG = zeros(5,1);

GG(1, 1) = 0;
GG(2, 1) = (m1 + m2 + m3 + m4 + m5)*gravi;
GG(3, 1) = (m3*(l2*cos(th1 + th2) + l1*cos(th1) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + l1*cos(th1) + (l5*cos(th1 + th2 + th5))/2) + m2*((l2*cos(th1 + th2))/2 + l1*cos(th1)) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3)) + (l1*m1*cos(th1))/2)*gravi;
GG(4, 1) = (m3*(l2*cos(th1 + th2) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + (l5*cos(th1 + th2 + th5))/2) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) + (l2*m2*cos(th1 + th2))/2)*gravi;
GG(5, 1) = (m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l3*cos(th1 + th2 + th3)) + (l3*m3*cos(th1 + th2 + th3))/2)*gravi;
GG(6, 1) = ((l4*m4*cos(th1 + th2 + th3 + th4))/2)*gravi;
GG(7, 1) = ((l5*m5*cos(th1 + th2 + th5))/2)*gravi;

%% input torques mapping matrix
S = [
    0,0,0,0,0;
    0,0,0,0,0;
    1,0,0,0,0;
    0,1,0,0,0;
    0,0,1,0,0;
    0,0,0,1,0;
    0,0,-1,0,1;];

%% Biarticular Spring Force

% Calculating the knee and hip angles
% We meed wrapTo2Pi() because some angles increase to inf due to the reset
% function eg. th1 and th5

phi_h_st = mod(th5 - pi, 2*pi); % angle between torso and stance thigh
phi_k_st = mod(pi - th2, 2*pi); % angle between stance thigh and shank
phi_h_sw = mod(th5 - th3, 2*pi); % angle between torso and swing thigh
phi_k_sw = mod(th4 - pi, 2*pi); % angle between swing thigh and shank

% Calculating the elongation in the springs
delta_l_st = r_h*(phi_h_st - phi_h0) - r_k*(phi_k_st - phi_k0); % elongation of BA in stance leg [m]
delta_l_sw = r_h*(phi_h_sw - phi_h0) - r_k*(phi_k_sw - phi_k0); % elongation of BA in swing leg [m]

% torques due to BA in stance leg
tau_st = [
    0;
    0;
    0;
    -k_ba*delta_l_st*r_k;
    0;
    0;
    -k_ba*delta_l_st*r_h];

% torques due to BA in swing leg
tau_sw = [
    0;
    0;
    0;
    0;
    k_ba*delta_l_sw*r_h;
    k_ba*delta_l_sw*r_k;
    -k_ba*delta_l_sw*r_h];

springForce = tau_st + tau_sw;

%% Constraint Jacobians

% For single stance phase
J_c_ss = zeros(2,7);
J_c_ss(1,:) = [ 1, 0, 0, 0, 0, 0, 0];
J_c_ss(2,:) = [ 0, 1, 0, 0, 0, 0, 0];

% For double stance phase
J_c_ds = zeros(4,7);
J_c_ds(1,:) = [ 1, 0, 0, 0, 0, 0, 0];
J_c_ds(2,:) = [ 0, 1, 0, 0, 0, 0, 0];
J_c_ds(3,:) = [ 1, 0, - l4*sin(th1 + th2 + th3 + th4) - l2*sin(th1 + th2) - l1*sin(th1) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l2*sin(th1 + th2) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l3*sin(th1 + th2 + th3), -l4*sin(th1 + th2 + th3 + th4), 0];
J_c_ds(4,:) = [ 0, 1, l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3), l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3), l4*cos(th1 + th2 + th3 + th4) + l3*cos(th1 + th2 + th3), l4*cos(th1 + th2 + th3 + th4), 0];

%% Time derivatives of the constraint Jacobians

% For single stance phase
dJ_c_ss = zeros(2,7);
dJ_c_ss(1,:) = [ 0, 0, 0, 0, 0, 0, 0];
dJ_c_ss(2,:) = [ 0, 0, 0, 0, 0, 0, 0];

% For double stance phase
dJ_c_ds = zeros(4,7);
dJ_c_ds(1,:) = [ 0, 0, 0, 0, 0, 0, 0];
dJ_c_ds(2,:) = [ 0, 0, 0, 0, 0, 0, 0];
dJ_c_ds(3,:) = [ 0, 0, - dth3*(l4*cos(th1 + th2 + th3 + th4) + l3*cos(th1 + th2 + th3)) - dth1*(l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3)) - dth2*(l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) - dth4*l4*cos(th1 + th2 + th3 + th4), - dth3*(l4*cos(th1 + th2 + th3 + th4) + l3*cos(th1 + th2 + th3)) - dth1*(l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) - dth2*(l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) - dth4*l4*cos(th1 + th2 + th3 + th4), - dth1*(l4*cos(th1 + th2 + th3 + th4) + l3*cos(th1 + th2 + th3)) - dth2*(l4*cos(th1 + th2 + th3 + th4) + l3*cos(th1 + th2 + th3)) - dth3*(l4*cos(th1 + th2 + th3 + th4) + l3*cos(th1 + th2 + th3)) - dth4*l4*cos(th1 + th2 + th3 + th4), -l4*cos(th1 + th2 + th3 + th4)*(dth1 + dth2 + dth3 + dth4), 0];
dJ_c_ds(4,:) = [ 0, 0, - dth2*(l4*sin(th1 + th2 + th3 + th4) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)) - dth3*(l4*sin(th1 + th2 + th3 + th4) + l3*sin(th1 + th2 + th3)) - dth1*(l4*sin(th1 + th2 + th3 + th4) + l2*sin(th1 + th2) + l1*sin(th1) + l3*sin(th1 + th2 + th3)) - dth4*l4*sin(th1 + th2 + th3 + th4), - dth1*(l4*sin(th1 + th2 + th3 + th4) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)) - dth2*(l4*sin(th1 + th2 + th3 + th4) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)) - dth3*(l4*sin(th1 + th2 + th3 + th4) + l3*sin(th1 + th2 + th3)) - dth4*l4*sin(th1 + th2 + th3 + th4), - dth1*(l4*sin(th1 + th2 + th3 + th4) + l3*sin(th1 + th2 + th3)) - dth2*(l4*sin(th1 + th2 + th3 + th4) + l3*sin(th1 + th2 + th3)) - dth3*(l4*sin(th1 + th2 + th3 + th4) + l3*sin(th1 + th2 + th3)) - dth4*l4*sin(th1 + th2 + th3 + th4), -l4*sin(th1 + th2 + th3 + th4)*(dth1 + dth2 + dth3 + dth4), 0];

%% constraint force

lambda_ss = -inv(J_c_ss*inv(MM)*J_c_ss')*(J_c_ss*inv(MM)*(S*U+tau_st+tau_sw-CC-GG)+dJ_c_ss*dq);

lambda_ds = -inv(J_c_ds*inv(MM)*J_c_ds')*(J_c_ds*inv(MM)*(S*U+tau_st+tau_sw-CC-GG)+dJ_c_ds*dq);

%% GRF output

if f_SS == 1
    GRF = [lambda_ss; 0; 0];
else
    GRF = lambda_ds;
end

%% Acceleration Calculation
ddq_GRF = zeros(7,1);

switch_time = flag(4); % time when double stance phase begins
if (f_SS == -1 && lambda_ds(4) >= 0) || (f_SS == -1 && t - switch_time < 0.05) % second part is because right after impact, there might be some fluctuations so it disregards them for 0.01 secs
% if f_SS == 0
    ddq_GRF = inv(MM)*(- CC - GG + S*U + tau_sw + tau_st + transpose(J_c_ds)*lambda_ds);  
else
    ddq_GRF = inv(MM)*(- CC - GG + S*U + tau_sw + tau_st + transpose(J_c_ss)*lambda_ss);  
end
end

