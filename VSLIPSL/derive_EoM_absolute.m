% Script to derive equation of motion, for absolute coordinates system 5
% Link Robot
% 2019.12.16
% PELIT Mustafa Melih

clear all
close all

% !!!! absolute angles !!!!
syms th1 th2 th3 th4 th5 real
syms dth1 dth2 dth3 dth4 dth5 real
syms ddth1 ddth2 ddth3 ddth4 ddth5 real
syms m1 m2 m3 m4 m5 real
syms I1 I2 I3 I4 I5 real
syms l1 l2 l3 l4 l5 real
syms gravi real

q = [th1; th2; th3; th4; th5];
dq = [dth1; dth2; dth3; dth4; dth5];
ddq = [ddth1; ddth2; ddth3; ddth4; ddth5];

m = [m1; m2; m3; m4; m5];

% CoM of Links
xG = [
    (l1/2)*cos(th1);
    l1*cos(th1) + (l2/2)*cos(th2);
    l1*cos(th1) + l2*cos(th2) + (l3/2)*cos(th3);
    l1*cos(th1) + l2*cos(th2) + l3*cos(th3) + (l4/2)*cos(th4);
    l1*cos(th1) + l2*cos(th2) + (l5/2)*cos(th5)];

zG = [
    (l1/2)*sin(th1);
    l1*sin(th1) + (l2/2)*sin(th2);
    l1*sin(th1) + l2*sin(th2) + (l3/2)*sin(th3);
    l1*sin(th1) + l2*sin(th2) + l3*sin(th3) + (l4/2)*sin(th4);
    l1*sin(th1) + l2*sin(th2) + (l5/2)*sin(th5)];

% Velocities of links' CoMs
for i=1:length(q)
    dxG(i) = jacobian(xG(i), q)*dq;
    dzG(i) = jacobian(zG(i), q)*dq;
end

% Calculating kinetic energy due to CoM velocities and potential energies
KE_vel = 0;
PE = 0;
for i=1:length(q)
    KE_vel = KE_vel + 0.5*m(i)*(dxG(i)^2 + dzG(i)^2);
    PE = PE + m(i)*gravi*zG(i);
end

KE_inertia = 0.5*(I1*dth1^2 + I2*dth2^2 + I3*dth3^2 + I4*dth4^2 + I5*dth5^2);
KE = simplify(KE_vel + KE_inertia); % total KE
PE = simplify(PE);

% Calculating Lagrangian
Lag =  simplify(KE - PE);
q_dq = [q; dq];
dq_ddq = [dq; ddq];

for i = 1:1:length(q)
Ldq(i,:) = diff(Lag,dq(i,:));
Ldq_dt(i,:) = jacobian(Ldq(i,:),q_dq)*dq_ddq;
Lq(i,:) = diff(Lag,q(i,:));
tau(i,:)= Ldq_dt(i,:) - Lq(i,:);
end

M = simplify(jacobian(tau,ddq));
G = simplify(collect(tau-subs(tau,gravi,0),gravi));
M_line = M*ddq;
C = simplify(tau-M_line-G);

fprintf('-----------inertia_matrix-------------\n\n')
for i=1:1:length(q)
    for j= 1:1:length(q)
        fprintf('M%d%d', i ,j)
        M(i,j)
    end
end
fprintf('-----------Coliori and centrifugal force matrix (vector)-------------\n\n')
for i=1:1:length(q)
    fprintf('C%d', i)
    C(i,:)
end
 fprintf('-----------gravitational_vector-----------\n\n')
for i=1:1:length(q)
    fprintf('G%d', i)
    G(i,:)
end

