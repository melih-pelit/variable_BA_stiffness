function U_test = ZMP_control(t, X, param, flag, U)

% 2021.09.17 function for calculating the u_1 (ankle torque) compatible
% with the ZMP condition

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

%% flags
f_SS = flag(1); % indicates which stance phase it is
foot = flag(2); % location of the stance foot
foot_prev = flag(3); % previous location of the stance foot


%% Inertia Matrix
MM = MM_matrix_relative_floating(X, param, flag);

%% Centrifugal force · Coriolis force Vector
CC = zeros(7,1);

CC(1, 1) = - (dth1^2*l3*m3*cos(th1 + th2 + th3))/2 - dth1^2*l3*m4*cos(th1 + th2 + th3) - (dth2^2*l3*m3*cos(th1 + th2 + th3))/2 - dth2^2*l3*m4*cos(th1 + th2 + th3) - (dth3^2*l3*m3*cos(th1 + th2 + th3))/2 - dth3^2*l3*m4*cos(th1 + th2 + th3) - (dth1^2*l5*m5*cos(th1 + th2 + th5))/2 - (dth2^2*l5*m5*cos(th1 + th2 + th5))/2 - (dth5^2*l5*m5*cos(th1 + th2 + th5))/2 - (dth1^2*l4*m4*cos(th1 + th2 + th3 + th4))/2 - (dth2^2*l4*m4*cos(th1 + th2 + th3 + th4))/2 - (dth3^2*l4*m4*cos(th1 + th2 + th3 + th4))/2 - (dth4^2*l4*m4*cos(th1 + th2 + th3 + th4))/2 - (dth1^2*l2*m2*cos(th1 + th2))/2 - dth1^2*l2*m3*cos(th1 + th2) - (dth2^2*l2*m2*cos(th1 + th2))/2 - dth1^2*l2*m4*cos(th1 + th2) - dth2^2*l2*m3*cos(th1 + th2) - dth1^2*l2*m5*cos(th1 + th2) - dth2^2*l2*m4*cos(th1 + th2) - dth2^2*l2*m5*cos(th1 + th2) - (dth1^2*l1*m1*cos(th1))/2 - dth1^2*l1*m2*cos(th1) - dth1^2*l1*m3*cos(th1) - dth1^2*l1*m4*cos(th1) - dth1^2*l1*m5*cos(th1) - dth1*dth2*l3*m3*cos(th1 + th2 + th3) - 2*dth1*dth2*l3*m4*cos(th1 + th2 + th3) - dth1*dth3*l3*m3*cos(th1 + th2 + th3) - 2*dth1*dth3*l3*m4*cos(th1 + th2 + th3) - dth2*dth3*l3*m3*cos(th1 + th2 + th3) - 2*dth2*dth3*l3*m4*cos(th1 + th2 + th3) - dth1*dth2*l5*m5*cos(th1 + th2 + th5) - dth1*dth5*l5*m5*cos(th1 + th2 + th5) - dth2*dth5*l5*m5*cos(th1 + th2 + th5) - dth1*dth2*l4*m4*cos(th1 + th2 + th3 + th4) - dth1*dth3*l4*m4*cos(th1 + th2 + th3 + th4) - dth1*dth4*l4*m4*cos(th1 + th2 + th3 + th4) - dth2*dth3*l4*m4*cos(th1 + th2 + th3 + th4) - dth2*dth4*l4*m4*cos(th1 + th2 + th3 + th4) - dth3*dth4*l4*m4*cos(th1 + th2 + th3 + th4) - dth1*dth2*l2*m2*cos(th1 + th2) - 2*dth1*dth2*l2*m3*cos(th1 + th2) - 2*dth1*dth2*l2*m4*cos(th1 + th2) - 2*dth1*dth2*l2*m5*cos(th1 + th2);
CC(2, 1) = - (dth1^2*l3*m3*sin(th1 + th2 + th3))/2 - dth1^2*l3*m4*sin(th1 + th2 + th3) - (dth2^2*l3*m3*sin(th1 + th2 + th3))/2 - dth2^2*l3*m4*sin(th1 + th2 + th3) - (dth3^2*l3*m3*sin(th1 + th2 + th3))/2 - dth3^2*l3*m4*sin(th1 + th2 + th3) - (dth1^2*l5*m5*sin(th1 + th2 + th5))/2 - (dth2^2*l5*m5*sin(th1 + th2 + th5))/2 - (dth5^2*l5*m5*sin(th1 + th2 + th5))/2 - (dth1^2*l4*m4*sin(th1 + th2 + th3 + th4))/2 - (dth2^2*l4*m4*sin(th1 + th2 + th3 + th4))/2 - (dth3^2*l4*m4*sin(th1 + th2 + th3 + th4))/2 - (dth4^2*l4*m4*sin(th1 + th2 + th3 + th4))/2 - (dth1^2*l2*m2*sin(th1 + th2))/2 - dth1^2*l2*m3*sin(th1 + th2) - (dth2^2*l2*m2*sin(th1 + th2))/2 - dth1^2*l2*m4*sin(th1 + th2) - dth2^2*l2*m3*sin(th1 + th2) - dth1^2*l2*m5*sin(th1 + th2) - dth2^2*l2*m4*sin(th1 + th2) - dth2^2*l2*m5*sin(th1 + th2) - (dth1^2*l1*m1*sin(th1))/2 - dth1^2*l1*m2*sin(th1) - dth1^2*l1*m3*sin(th1) - dth1^2*l1*m4*sin(th1) - dth1^2*l1*m5*sin(th1) - dth1*dth2*l3*m3*sin(th1 + th2 + th3) - 2*dth1*dth2*l3*m4*sin(th1 + th2 + th3) - dth1*dth3*l3*m3*sin(th1 + th2 + th3) - 2*dth1*dth3*l3*m4*sin(th1 + th2 + th3) - dth2*dth3*l3*m3*sin(th1 + th2 + th3) - 2*dth2*dth3*l3*m4*sin(th1 + th2 + th3) - dth1*dth2*l5*m5*sin(th1 + th2 + th5) - dth1*dth5*l5*m5*sin(th1 + th2 + th5) - dth2*dth5*l5*m5*sin(th1 + th2 + th5) - dth1*dth2*l4*m4*sin(th1 + th2 + th3 + th4) - dth1*dth3*l4*m4*sin(th1 + th2 + th3 + th4) - dth1*dth4*l4*m4*sin(th1 + th2 + th3 + th4) - dth2*dth3*l4*m4*sin(th1 + th2 + th3 + th4) - dth2*dth4*l4*m4*sin(th1 + th2 + th3 + th4) - dth3*dth4*l4*m4*sin(th1 + th2 + th3 + th4) - dth1*dth2*l2*m2*sin(th1 + th2) - 2*dth1*dth2*l2*m3*sin(th1 + th2) - 2*dth1*dth2*l2*m4*sin(th1 + th2) - 2*dth1*dth2*l2*m5*sin(th1 + th2);
CC(3, 1) = - (dth2^2*l1*l3*m3*sin(th2 + th3))/2 - dth2^2*l1*l3*m4*sin(th2 + th3) - (dth3^2*l1*l3*m3*sin(th2 + th3))/2 - dth3^2*l1*l3*m4*sin(th2 + th3) - (dth2^2*l1*l5*m5*sin(th2 + th5))/2 - (dth3^2*l2*l4*m4*sin(th3 + th4))/2 - (dth4^2*l2*l4*m4*sin(th3 + th4))/2 - (dth5^2*l1*l5*m5*sin(th2 + th5))/2 - (dth2^2*l1*l2*m2*sin(th2))/2 - dth2^2*l1*l2*m3*sin(th2) - dth2^2*l1*l2*m4*sin(th2) - dth2^2*l1*l2*m5*sin(th2) - (dth3^2*l2*l3*m3*sin(th3))/2 - dth3^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 - (dth5^2*l2*l5*m5*sin(th5))/2 - (dth2^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - (dth3^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - (dth4^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - dth1*dth2*l1*l3*m3*sin(th2 + th3) - 2*dth1*dth2*l1*l3*m4*sin(th2 + th3) - dth1*dth3*l1*l3*m3*sin(th2 + th3) - 2*dth1*dth3*l1*l3*m4*sin(th2 + th3) - dth2*dth3*l1*l3*m3*sin(th2 + th3) - 2*dth2*dth3*l1*l3*m4*sin(th2 + th3) - dth1*dth2*l1*l5*m5*sin(th2 + th5) - dth1*dth3*l2*l4*m4*sin(th3 + th4) - dth1*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth3*l2*l4*m4*sin(th3 + th4) - dth2*dth4*l2*l4*m4*sin(th3 + th4) - dth1*dth5*l1*l5*m5*sin(th2 + th5) - dth3*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth5*l1*l5*m5*sin(th2 + th5) - dth1*dth2*l1*l2*m2*sin(th2) - 2*dth1*dth2*l1*l2*m3*sin(th2) - 2*dth1*dth2*l1*l2*m4*sin(th2) - 2*dth1*dth2*l1*l2*m5*sin(th2) - dth1*dth3*l2*l3*m3*sin(th3) - 2*dth1*dth3*l2*l3*m4*sin(th3) - dth2*dth3*l2*l3*m3*sin(th3) - 2*dth2*dth3*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4) - dth1*dth5*l2*l5*m5*sin(th5) - dth2*dth5*l2*l5*m5*sin(th5) - dth1*dth2*l1*l4*m4*sin(th2 + th3 + th4) - dth1*dth3*l1*l4*m4*sin(th2 + th3 + th4) - dth1*dth4*l1*l4*m4*sin(th2 + th3 + th4) - dth2*dth3*l1*l4*m4*sin(th2 + th3 + th4) - dth2*dth4*l1*l4*m4*sin(th2 + th3 + th4) - dth3*dth4*l1*l4*m4*sin(th2 + th3 + th4);
CC(4, 1) = (dth1^2*l1*l3*m3*sin(th2 + th3))/2 + dth1^2*l1*l3*m4*sin(th2 + th3) + (dth1^2*l1*l5*m5*sin(th2 + th5))/2 - (dth3^2*l2*l4*m4*sin(th3 + th4))/2 - (dth4^2*l2*l4*m4*sin(th3 + th4))/2 + (dth1^2*l1*l2*m2*sin(th2))/2 + dth1^2*l1*l2*m3*sin(th2) + dth1^2*l1*l2*m4*sin(th2) + dth1^2*l1*l2*m5*sin(th2) - (dth3^2*l2*l3*m3*sin(th3))/2 - dth3^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 - (dth5^2*l2*l5*m5*sin(th5))/2 + (dth1^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - dth1*dth3*l2*l4*m4*sin(th3 + th4) - dth1*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth3*l2*l4*m4*sin(th3 + th4) - dth2*dth4*l2*l4*m4*sin(th3 + th4) - dth3*dth4*l2*l4*m4*sin(th3 + th4) - dth1*dth3*l2*l3*m3*sin(th3) - 2*dth1*dth3*l2*l3*m4*sin(th3) - dth2*dth3*l2*l3*m3*sin(th3) - 2*dth2*dth3*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4) - dth1*dth5*l2*l5*m5*sin(th5) - dth2*dth5*l2*l5*m5*sin(th5);
CC(5, 1) = (dth1^2*l1*l3*m3*sin(th2 + th3))/2 + dth1^2*l1*l3*m4*sin(th2 + th3) + (dth1^2*l2*l4*m4*sin(th3 + th4))/2 + (dth2^2*l2*l4*m4*sin(th3 + th4))/2 + (dth1^2*l2*l3*m3*sin(th3))/2 + dth1^2*l2*l3*m4*sin(th3) + (dth2^2*l2*l3*m3*sin(th3))/2 + dth2^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 + (dth1^2*l1*l4*m4*sin(th2 + th3 + th4))/2 + dth1*dth2*l2*l4*m4*sin(th3 + th4) + dth1*dth2*l2*l3*m3*sin(th3) + 2*dth1*dth2*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4);
CC(6, 1) = (l4*m4*(dth1^2*l2*sin(th3 + th4) + dth2^2*l2*sin(th3 + th4) + dth1^2*l3*sin(th4) + dth2^2*l3*sin(th4) + dth3^2*l3*sin(th4) + dth1^2*l1*sin(th2 + th3 + th4) + 2*dth1*dth2*l2*sin(th3 + th4) + 2*dth1*dth2*l3*sin(th4) + 2*dth1*dth3*l3*sin(th4) + 2*dth2*dth3*l3*sin(th4)))/2;
CC(7, 1) = (l5*m5*(dth1^2*l1*sin(th2 + th5) + dth1^2*l2*sin(th5) + dth2^2*l2*sin(th5) + 2*dth1*dth2*l2*sin(th5)))/2;

%% Gravity Vector
GG = zeros(7,1);

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

%% Calculating lambda_y (vertical GRF on the foot with the ankle torque)

lambda_ss = -inv(J_c_ss*inv(MM)*J_c_ss')*(J_c_ss*inv(MM)*(S*U-CC-GG)+dJ_c_ss*dq);

lambda_ds = -inv(J_c_ds*inv(MM)*J_c_ds')*(J_c_ds*inv(MM)*(S*U-CC-GG)+dJ_c_ds*dq);

if f_SS == 1
    GRF = [lambda_ss; 0; 0];
else
    GRF = lambda_ds;
end

lambda_y = GRF(2);

%% Calculating x_ZMP position

m_foot = 2;
x_foot_CoM = 0.1;
F_y_GRF = -(-m_foot*gravi + -lambda_y);
x_ZMP = (U(1) + x_foot_CoM*m_foot*gravi)/F_y_GRF;

if x_ZMP > 0.3
    
    if f_SS == 1
        F_ZMP = @(u1) ([0, 1]*(-inv(J_c_ss*inv(MM)*J_c_ss')*(J_c_ss*inv(MM)*(S*[u1; U(2); U(3); U(4); U(5)]-CC-GG)+dJ_c_ss*dq)))*0.3 - u1 - x_foot_CoM*m_foot*gravi;
        
        u1_ZMP = fsolve(F_ZMP,0,optimoptions('fsolve','Algorithm', 'Levenberg-Marquardt'));
        
        % u1_ZMP = 0;
    else
        u1_ZMP = 0;
    end
elseif x_ZMP < -0.2
    
    if f_SS == 1
        
        F_ZMP = @(u1) ([0, 1]*(-inv(J_c_ss*inv(MM)*J_c_ss')*(J_c_ss*inv(MM)*(S*[u1; U(2); U(3); U(4); U(5)]-CC-GG)+dJ_c_ss*dq)))*(-0.1) - u1 - x_foot_CoM*m_foot*gravi;
        
        u1_ZMP = fsolve(F_ZMP,0,optimoptions('fsolve','Algorithm', 'Levenberg-Marquardt'));
    else
        u1_ZMP = 0;
    end
else
    u1_ZMP = U(1);    
    
end

U_test = [u1_ZMP; U(2); U(3); U(4); U(5)];

end