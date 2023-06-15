function MM = MM_matrix(X,param)
% MM matrix for SLIP walking robot
% 09.08.2018

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

q = [th1;th2;th3;th4;th5];
dq = [dth1;dth2;dth3;dth4;dth5];

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
MM = zeros(5);

MM(1, 1) = I1 + I2 + I3 + I4 + I5 + (l1^2*m1)/4 + l1^2*m2 + l1^2*m3 + (l2^2*m2)/4 + l1^2*m4 + l2^2*m3 + l1^2*m5 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l1*l3*m3*cos(th2 + th3) + 2*l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + l1*l5*m5*cos(th2 + th5) + l1*l2*m2*cos(th2) + 2*l1*l2*m3*cos(th2) + 2*l1*l2*m4*cos(th2) + 2*l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + l1*l4*m4*cos(th2 + th3 + th4);
MM(1, 2) = I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(1, 3) = I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(1, 4) = I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(1, 5) = I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2;
MM(2, 1) = I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(2, 2) = I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l2*l4*m4*cos(th3 + th4) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5);
MM(2, 3) = I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4);
MM(2, 4) = I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2;
MM(2, 5) = (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5;
MM(3, 1) = I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(3, 2) = I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4);
MM(3, 3) = I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + l3*l4*m4*cos(th4);
MM(3, 4) = (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4;
MM(3, 5) = 0;
MM(4, 1) = I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2;
MM(4, 2) = I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2;
MM(4, 3) = (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4;
MM(4, 4) = (m4*l4^2)/4 + I4;
MM(4, 5) = 0;
MM(5, 1) = I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2;
MM(5, 2) = (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5;
MM(5, 3) = 0;
MM(5, 4) = 0;
MM(5, 5) = (m5*l5^2)/4 + I5;

end