function MM = MM_matrix_relative_floating(X, param, flag)
% MM matrix for SLIP walking robot (relative angle /w floating coord.)
% 2020.10.22

th1 = X(1);
th2 = X(2);
th3 = X(3);
th4 = X(4);
th5 = X(5);

dth1 = X(6);
dth2 = X(7);
dth3 = X(8);
dth4 = X(9);
dth5 = X(10);

x_st = flag(4); % x position of the stance foot
y_st = 0;

dx_st = 0;
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

%% MM Matrix
MM = zeros(7);

MM(1, 1) = m1 + m2 + m3 + m4 + m5;
MM(1, 2) = 0;
MM(1, 3) = - (m3*(2*l2*sin(th1 + th2) + 2*l1*sin(th1) + l3*sin(th1 + th2 + th3)))/2 - (m5*(2*l2*sin(th1 + th2) + 2*l1*sin(th1) + l5*sin(th1 + th2 + th5)))/2 - (m2*(l2*sin(th1 + th2) + 2*l1*sin(th1)))/2 - (m4*(l4*sin(th1 + th2 + th3 + th4) + 2*l2*sin(th1 + th2) + 2*l1*sin(th1) + 2*l3*sin(th1 + th2 + th3)))/2 - (l1*m1*sin(th1))/2;
MM(1, 4) = - (m4*(l4*sin(th1 + th2 + th3 + th4) + 2*l2*sin(th1 + th2) + 2*l3*sin(th1 + th2 + th3)))/2 - (m3*(2*l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)))/2 - (m5*(2*l2*sin(th1 + th2) + l5*sin(th1 + th2 + th5)))/2 - (l2*m2*sin(th1 + th2))/2;
MM(1, 5) = - (m4*(l4*sin(th1 + th2 + th3 + th4) + 2*l3*sin(th1 + th2 + th3)))/2 - (l3*m3*sin(th1 + th2 + th3))/2;
MM(1, 6) = -(l4*m4*sin(th1 + th2 + th3 + th4))/2;
MM(1, 7) = -(l5*m5*sin(th1 + th2 + th5))/2;

MM(2, 1) = 0;
MM(2, 2) = m1 + m2 + m3 + m4 + m5;
MM(2, 3) = (m3*(2*l2*cos(th1 + th2) + 2*l1*cos(th1) + l3*cos(th1 + th2 + th3)))/2 + (m5*(2*l2*cos(th1 + th2) + 2*l1*cos(th1) + l5*cos(th1 + th2 + th5)))/2 + (m2*(l2*cos(th1 + th2) + 2*l1*cos(th1)))/2 + (m4*(l4*cos(th1 + th2 + th3 + th4) + 2*l2*cos(th1 + th2) + 2*l1*cos(th1) + 2*l3*cos(th1 + th2 + th3)))/2 + (l1*m1*cos(th1))/2;
MM(2, 4) = (m3*(2*l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)))/2 + (m5*(2*l2*cos(th1 + th2) + l5*cos(th1 + th2 + th5)))/2 + (m4*(l4*cos(th1 + th2 + th3 + th4) + 2*l2*cos(th1 + th2) + 2*l3*cos(th1 + th2 + th3)))/2 + (l2*m2*cos(th1 + th2))/2;
MM(2, 5) = (m4*(l4*cos(th1 + th2 + th3 + th4) + 2*l3*cos(th1 + th2 + th3)))/2 + (l3*m3*cos(th1 + th2 + th3))/2;
MM(2, 6) = (l4*m4*cos(th1 + th2 + th3 + th4))/2;
MM(2, 7) = (l5*m5*cos(th1 + th2 + th5))/2;

MM(3, 1) = - (m3*(2*l2*sin(th1 + th2) + 2*l1*sin(th1) + l3*sin(th1 + th2 + th3)))/2 - (m5*(2*l2*sin(th1 + th2) + 2*l1*sin(th1) + l5*sin(th1 + th2 + th5)))/2 - (m2*(l2*sin(th1 + th2) + 2*l1*sin(th1)))/2 - (m4*(l4*sin(th1 + th2 + th3 + th4) + 2*l2*sin(th1 + th2) + 2*l1*sin(th1) + 2*l3*sin(th1 + th2 + th3)))/2 - (l1*m1*sin(th1))/2;
MM(3, 2) = (m3*(2*l2*cos(th1 + th2) + 2*l1*cos(th1) + l3*cos(th1 + th2 + th3)))/2 + (m5*(2*l2*cos(th1 + th2) + 2*l1*cos(th1) + l5*cos(th1 + th2 + th5)))/2 + (m2*(l2*cos(th1 + th2) + 2*l1*cos(th1)))/2 + (m4*(l4*cos(th1 + th2 + th3 + th4) + 2*l2*cos(th1 + th2) + 2*l1*cos(th1) + 2*l3*cos(th1 + th2 + th3)))/2 + (l1*m1*cos(th1))/2;
MM(3, 3) = I1 + I2 + I3 + I4 + I5 + (l1^2*m1)/4 + l1^2*m2 + l1^2*m3 + (l2^2*m2)/4 + l1^2*m4 + l2^2*m3 + l1^2*m5 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l1*l3*m3*cos(th2 + th3) + 2*l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + l1*l5*m5*cos(th2 + th5) + l1*l2*m2*cos(th2) + 2*l1*l2*m3*cos(th2) + 2*l1*l2*m4*cos(th2) + 2*l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + l1*l4*m4*cos(th2 + th3 + th4);
MM(3, 4) = I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(3, 5) = I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(3, 6) = I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(3, 7) = I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2;

MM(4, 1) = - (m4*(l4*sin(th1 + th2 + th3 + th4) + 2*l2*sin(th1 + th2) + 2*l3*sin(th1 + th2 + th3)))/2 - (m3*(2*l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)))/2 - (m5*(2*l2*sin(th1 + th2) + l5*sin(th1 + th2 + th5)))/2 - (l2*m2*sin(th1 + th2))/2;
MM(4, 2) = (m3*(2*l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)))/2 + (m5*(2*l2*cos(th1 + th2) + l5*cos(th1 + th2 + th5)))/2 + (m4*(l4*cos(th1 + th2 + th3 + th4) + 2*l2*cos(th1 + th2) + 2*l3*cos(th1 + th2 + th3)))/2 + (l2*m2*cos(th1 + th2))/2;
MM(4, 3) = I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(4, 4) = I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l2*l4*m4*cos(th3 + th4) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5);
MM(4, 5) = I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4);
MM(4, 6) = I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2;
MM(4, 7) = (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5;

MM(5, 1) = - (m4*(l4*sin(th1 + th2 + th3 + th4) + 2*l3*sin(th1 + th2 + th3)))/2 - (l3*m3*sin(th1 + th2 + th3))/2;
MM(5, 2) = (m4*(l4*cos(th1 + th2 + th3 + th4) + 2*l3*cos(th1 + th2 + th3)))/2 + (l3*m3*cos(th1 + th2 + th3))/2;
MM(5, 3) = I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(5, 4) = I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4);
MM(5, 5) = I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + l3*l4*m4*cos(th4);
MM(5, 6) = (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4;
MM(5, 7) = 0;

MM(6,1) = -(l4*m4*sin(th1 + th2 + th3 + th4))/2;
MM(6,2) = (l4*m4*cos(th1 + th2 + th3 + th4))/2;
MM(6,3) = I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(6,4) = I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2;
MM(6,5) = (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4;
MM(6,6) = (m4*l4^2)/4 + I4;
MM(6,7) = 0;

MM(7,1) = -(l5*m5*sin(th1 + th2 + th5))/2;
MM(7,2) = (l5*m5*cos(th1 + th2 + th5))/2;
MM(7,3) = I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2;
MM(7,4) = (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5;
MM(7,5) = 0;
MM(7,6) = 0;
MM(7,7) = (m5*l5^2)/4 + I5;

end