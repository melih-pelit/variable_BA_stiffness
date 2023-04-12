function [J_t_DS, dJ_t_DS] = calculate_jacobian_DS(X,param)
% values gotten from derive_Jacobian.m

%% generalized coordinates and their derivatives
% th1 = X(1);
% th2 = X(2);
% th3 = X(3);
% th4 = X(4);
% th5 = X(5);
% 
% dth1 = X(6);
% dth2 = X(7);
% dth3 = X(8);
% dth4 = X(9);
% dth5 = X(10);

%% variables (absolute angles)
th1 = X(1);
th2 = X(1) + X(2);
th3 = X(1) + X(2) + X(3);
th4 = X(1) + X(2) + X(3) + X(4);
th5 = X(1) + X(2) + X(5);

dth1 = X(6);
dth2 = X(6) + X(7);
dth3 = X(6) + X(7) + X(8);
dth4 = X(6) + X(7) + X(8) + X(9);
dth5 = X(6) + X(7) + X(10);

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

%% Jacobian
J_t_DS = zeros(3,5);
% 
% J_t_DS(1,1) = -(m3*(l2*sin(th1 + th2) + l1*sin(th1) + (l3*sin(th1 + th2 + th3))/2) + m5*(l2*sin(th1 + th2) + l1*sin(th1) + (l5*sin(th1 + th2 + th5))/2) + m2*((l1*sin(th1 + th2))/2 + l1*sin(th1)) + m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l2*sin(th1 + th2) + l1*sin(th1) + l3*sin(th1 + th2 + th3)) + (l1*m1*sin(th1))/2)/(m1 + m2 + m3 + m4 + m5);
% J_t_DS(1,2) = -(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)) + m3*(l2*sin(th1 + th2) + (l3*sin(th1 + th2 + th3))/2) + m5*(l2*sin(th1 + th2) + (l5*sin(th1 + th2 + th5))/2) + (l1*m2*sin(th1 + th2))/2)/(m1 + m2 + m3 + m4 + m5);
% J_t_DS(1,3) = -(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l3*sin(th1 + th2 + th3)) + (l3*m3*sin(th1 + th2 + th3))/2)/(m1 + m2 + m3 + m4 + m5);
% J_t_DS(1,4) = -(l4*m4*sin(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5));
% J_t_DS(1,5) = -(l5*m5*sin(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5));
% 
% J_t_DS(2,1) = (m3*(l2*cos(th1 + th2) + l1*cos(th1) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + l1*cos(th1) + (l5*cos(th1 + th2 + th5))/2) + m2*((l2*cos(th1 + th2))/2 + l1*cos(th1)) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3)) + (l1*m1*cos(th1))/2)/(m1 + m2 + m3 + m4 + m5);
% J_t_DS(2,2) = (m3*(l2*cos(th1 + th2) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + (l5*cos(th1 + th2 + th5))/2) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) + (l2*m2*cos(th1 + th2))/2)/(m1 + m2 + m3 + m4 + m5);
% J_t_DS(2,3) = (m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l3*cos(th1 + th2 + th3)) + (l3*m3*cos(th1 + th2 + th3))/2)/(m1 + m2 + m3 + m4 + m5);
% J_t_DS(2,4) = (l4*m4*cos(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5));
% J_t_DS(2,5) = (l5*m5*cos(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5));
% 
% J_t_DS(3,1) = 0;
% J_t_DS(3,2) = 0;
% J_t_DS(3,3) = 0;
% J_t_DS(3,4) = 0;
% J_t_DS(3,5) = 1;

% absolute angle version
J_t_DS(1,1) = -((l1*m1*sin(th1))/2 + l1*m2*sin(th1) + l1*m3*sin(th1) + l1*m4*sin(th1) + l1*m5*sin(th1))/(m1 + m2 + m3 + m4 + m5);
J_t_DS(1,2) = -((l1*m2*sin(th2))/2 + l2*m3*sin(th2) + l2*m4*sin(th2) + l2*m5*sin(th2))/(m1 + m2 + m3 + m4 + m5);
J_t_DS(1,3) = -((l3*m3*sin(th3))/2 + l3*m4*sin(th3))/(m1 + m2 + m3 + m4 + m5);
J_t_DS(1,4) = -(l4*m4*sin(th4))/(2*(m1 + m2 + m3 + m4 + m5));
J_t_DS(1,5) = -(l5*m5*sin(th5))/(2*(m1 + m2 + m3 + m4 + m5));

J_t_DS(2,1) = ((l1*m1*cos(th1))/2 + l1*m2*cos(th1) + l1*m3*cos(th1) + l1*m4*cos(th1) + l1*m5*cos(th1))/(m1 + m2 + m3 + m4 + m5);
J_t_DS(2,2) = ((l2*m2*cos(th2))/2 + l2*m3*cos(th2) + l2*m4*cos(th2) + l2*m5*cos(th2))/(m1 + m2 + m3 + m4 + m5);
J_t_DS(2,3) = ((l3*m3*cos(th3))/2 + l3*m4*cos(th3))/(m1 + m2 + m3 + m4 + m5);
J_t_DS(2,4) = (l4*m4*cos(th4))/(2*(m1 + m2 + m3 + m4 + m5));
J_t_DS(2,5) = (l5*m5*cos(th5))/(2*(m1 + m2 + m3 + m4 + m5));

J_t_DS(3,1) = 0;
J_t_DS(3,2) = 0;
J_t_DS(3,3) = 0;
J_t_DS(3,4) = 0;
J_t_DS(3,5) = 1;

%% Derivative of the Jacobian
dJ_t_DS = zeros(3,5);

% dJ_t_DS(1,1) = -(dth1*l4*m4*cos(th1 + th2 + th3 + th4) + dth2*l4*m4*cos(th1 + th2 + th3 + th4) + dth3*l4*m4*cos(th1 + th2 + th3 + th4) + dth4*l4*m4*cos(th1 + th2 + th3 + th4) + dth1*l1*m2*cos(th1 + th2) + dth2*l1*m2*cos(th1 + th2) + 2*dth1*l2*m3*cos(th1 + th2) + 2*dth1*l2*m4*cos(th1 + th2) + 2*dth2*l2*m3*cos(th1 + th2) + 2*dth1*l2*m5*cos(th1 + th2) + 2*dth2*l2*m4*cos(th1 + th2) + 2*dth2*l2*m5*cos(th1 + th2) + dth1*l1*m1*cos(th1) + 2*dth1*l1*m2*cos(th1) + 2*dth1*l1*m3*cos(th1) + 2*dth1*l1*m4*cos(th1) + 2*dth1*l1*m5*cos(th1) + dth1*l3*m3*cos(th1 + th2 + th3) + 2*dth1*l3*m4*cos(th1 + th2 + th3) + dth2*l3*m3*cos(th1 + th2 + th3) + 2*dth2*l3*m4*cos(th1 + th2 + th3) + dth3*l3*m3*cos(th1 + th2 + th3) + 2*dth3*l3*m4*cos(th1 + th2 + th3) + dth1*l5*m5*cos(th1 + th2 + th5) + dth2*l5*m5*cos(th1 + th2 + th5) + dth5*l5*m5*cos(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5));
% dJ_t_DS(1,2) = -(dth1*l4*m4*cos(th1 + th2 + th3 + th4) + dth2*l4*m4*cos(th1 + th2 + th3 + th4) + dth3*l4*m4*cos(th1 + th2 + th3 + th4) + dth4*l4*m4*cos(th1 + th2 + th3 + th4) + dth1*l1*m2*cos(th1 + th2) + dth2*l1*m2*cos(th1 + th2) + 2*dth1*l2*m3*cos(th1 + th2) + 2*dth1*l2*m4*cos(th1 + th2) + 2*dth2*l2*m3*cos(th1 + th2) + 2*dth1*l2*m5*cos(th1 + th2) + 2*dth2*l2*m4*cos(th1 + th2) + 2*dth2*l2*m5*cos(th1 + th2) + dth1*l3*m3*cos(th1 + th2 + th3) + 2*dth1*l3*m4*cos(th1 + th2 + th3) + dth2*l3*m3*cos(th1 + th2 + th3) + 2*dth2*l3*m4*cos(th1 + th2 + th3) + dth3*l3*m3*cos(th1 + th2 + th3) + 2*dth3*l3*m4*cos(th1 + th2 + th3) + dth1*l5*m5*cos(th1 + th2 + th5) + dth2*l5*m5*cos(th1 + th2 + th5) + dth5*l5*m5*cos(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5));
% dJ_t_DS(1,3) = -(dth1*l4*m4*cos(th1 + th2 + th3 + th4) + dth2*l4*m4*cos(th1 + th2 + th3 + th4) + dth3*l4*m4*cos(th1 + th2 + th3 + th4) + dth4*l4*m4*cos(th1 + th2 + th3 + th4) + dth1*l3*m3*cos(th1 + th2 + th3) + 2*dth1*l3*m4*cos(th1 + th2 + th3) + dth2*l3*m3*cos(th1 + th2 + th3) + 2*dth2*l3*m4*cos(th1 + th2 + th3) + dth3*l3*m3*cos(th1 + th2 + th3) + 2*dth3*l3*m4*cos(th1 + th2 + th3))/(2*(m1 + m2 + m3 + m4 + m5));
% dJ_t_DS(1,4) = -(l4*m4*cos(th1 + th2 + th3 + th4)*(dth1 + dth2 + dth3 + dth4))/(2*(m1 + m2 + m3 + m4 + m5));
% dJ_t_DS(1,5) = -(l5*m5*cos(th1 + th2 + th5)*(dth1 + dth2 + dth5))/(2*(m1 + m2 + m3 + m4 + m5));
% 
% dJ_t_DS(2,1) = -(dth1*l4*m4*sin(th1 + th2 + th3 + th4) + dth2*l4*m4*sin(th1 + th2 + th3 + th4) + dth3*l4*m4*sin(th1 + th2 + th3 + th4) + dth4*l4*m4*sin(th1 + th2 + th3 + th4) + dth1*l2*m2*sin(th1 + th2) + 2*dth1*l2*m3*sin(th1 + th2) + dth2*l2*m2*sin(th1 + th2) + 2*dth1*l2*m4*sin(th1 + th2) + 2*dth2*l2*m3*sin(th1 + th2) + 2*dth1*l2*m5*sin(th1 + th2) + 2*dth2*l2*m4*sin(th1 + th2) + 2*dth2*l2*m5*sin(th1 + th2) + dth1*l1*m1*sin(th1) + 2*dth1*l1*m2*sin(th1) + 2*dth1*l1*m3*sin(th1) + 2*dth1*l1*m4*sin(th1) + 2*dth1*l1*m5*sin(th1) + dth1*l3*m3*sin(th1 + th2 + th3) + 2*dth1*l3*m4*sin(th1 + th2 + th3) + dth2*l3*m3*sin(th1 + th2 + th3) + 2*dth2*l3*m4*sin(th1 + th2 + th3) + dth3*l3*m3*sin(th1 + th2 + th3) + 2*dth3*l3*m4*sin(th1 + th2 + th3) + dth1*l5*m5*sin(th1 + th2 + th5) + dth2*l5*m5*sin(th1 + th2 + th5) + dth5*l5*m5*sin(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5));
% dJ_t_DS(2,2) = -(dth1*l4*m4*sin(th1 + th2 + th3 + th4) + dth2*l4*m4*sin(th1 + th2 + th3 + th4) + dth3*l4*m4*sin(th1 + th2 + th3 + th4) + dth4*l4*m4*sin(th1 + th2 + th3 + th4) + dth1*l2*m2*sin(th1 + th2) + 2*dth1*l2*m3*sin(th1 + th2) + dth2*l2*m2*sin(th1 + th2) + 2*dth1*l2*m4*sin(th1 + th2) + 2*dth2*l2*m3*sin(th1 + th2) + 2*dth1*l2*m5*sin(th1 + th2) + 2*dth2*l2*m4*sin(th1 + th2) + 2*dth2*l2*m5*sin(th1 + th2) + dth1*l3*m3*sin(th1 + th2 + th3) + 2*dth1*l3*m4*sin(th1 + th2 + th3) + dth2*l3*m3*sin(th1 + th2 + th3) + 2*dth2*l3*m4*sin(th1 + th2 + th3) + dth3*l3*m3*sin(th1 + th2 + th3) + 2*dth3*l3*m4*sin(th1 + th2 + th3) + dth1*l5*m5*sin(th1 + th2 + th5) + dth2*l5*m5*sin(th1 + th2 + th5) + dth5*l5*m5*sin(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5));
% dJ_t_DS(2,3) = -(dth1*l4*m4*sin(th1 + th2 + th3 + th4) + dth2*l4*m4*sin(th1 + th2 + th3 + th4) + dth3*l4*m4*sin(th1 + th2 + th3 + th4) + dth4*l4*m4*sin(th1 + th2 + th3 + th4) + dth1*l3*m3*sin(th1 + th2 + th3) + 2*dth1*l3*m4*sin(th1 + th2 + th3) + dth2*l3*m3*sin(th1 + th2 + th3) + 2*dth2*l3*m4*sin(th1 + th2 + th3) + dth3*l3*m3*sin(th1 + th2 + th3) + 2*dth3*l3*m4*sin(th1 + th2 + th3))/(2*(m1 + m2 + m3 + m4 + m5));
% dJ_t_DS(2,4) = -(l4*m4*sin(th1 + th2 + th3 + th4)*(dth1 + dth2 + dth3 + dth4))/(2*(m1 + m2 + m3 + m4 + m5));
% dJ_t_DS(2,5) = -(l5*m5*sin(th1 + th2 + th5)*(dth1 + dth2 + dth5))/(2*(m1 + m2 + m3 + m4 + m5));
% 
% dJ_t_DS(3,:) = zeros(1,5);

% absolute angle version
dJ_t_DS(1,1) = -(dth1*l1*cos(th1)*(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5))/(2*(m1 + m2 + m3 + m4 + m5));
dJ_t_DS(1,2) = -(dth2*cos(th2)*((l1*m2)/2 + l2*m3 + l2*m4 + l2*m5))/(m1 + m2 + m3 + m4 + m5);
dJ_t_DS(1,3) = -(dth3*l3*cos(th3)*(m3 + 2*m4))/(2*(m1 + m2 + m3 + m4 + m5));
dJ_t_DS(1,4) = -(dth4*l4*m4*cos(th4))/(2*(m1 + m2 + m3 + m4 + m5));
dJ_t_DS(1,5) = -(dth5*l5*m5*cos(th5))/(2*(m1 + m2 + m3 + m4 + m5));

dJ_t_DS(2,1) = -(dth1*l1*sin(th1)*(m1 + 2*m2 + 2*m3 + 2*m4 + 2*m5))/(2*(m1 + m2 + m3 + m4 + m5));
dJ_t_DS(2,2) = -(dth2*l2*sin(th2)*(m2 + 2*m3 + 2*m4 + 2*m5))/(2*(m1 + m2 + m3 + m4 + m5));
dJ_t_DS(2,3) = -(dth3*l3*sin(th3)*(m3 + 2*m4))/(2*(m1 + m2 + m3 + m4 + m5));
dJ_t_DS(2,4) = -(dth4*l4*m4*sin(th4))/(2*(m1 + m2 + m3 + m4 + m5));
dJ_t_DS(2,5) = -(dth5*l5*m5*sin(th5))/(2*(m1 + m2 + m3 + m4 + m5));

dJ_t_DS(3,:) = zeros(1,5);


end