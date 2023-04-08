% 2023.04.02 Deriving constaint Jacobian for keeping swing foot on the
% ground during double stance phase.

clear
close all

syms th1 th2 th3 th4 th5 real
syms dth1 dth2 dth3 dth4 dth5 real
syms m1 m2 m3 m4 m5 real
syms l1 l2 l3 l4 l5 real
syms I1 I2 I3 I4 I5 real
syms gravi real

q = [th1; th2; th3; th4; th5];
dq = [dth1; dth2; dth3; dth4; dth5];

%% J_c

x_sw = l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + l4*cos(th1 + th2 + th3 + th4);
y_sw = l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + l4*sin(th1 + th2 + th3 + th4);

P_sw = [x_sw; y_sw];

J_c = jacobian(P_sw, q);

%% time derivative of J_c

jacobSize = size(J_c);
dJ_c = sym(size(J_c));

for i = 1:1:jacobSize(1)
    for j = 1:1:jacobSize(2)
        dJ_c(i,j) = simplify(jacobian(J_c(i, j), q) * dq);
    end
end
