function [specific_resistance] = calc_sr(sol)
%CALC_SR Function to calculate specific resistance from OpenOCL solution
% (modified for underactuated system)
%   Detailed explanation goes here

param.m1 = mean(sol.parameters.m1.value); % mass of link 1 [kg]
param.m2 = mean(sol.parameters.m2.value); % mass of link 2 [kg]
param.m3 = mean(sol.parameters.m2.value); % mass of link 3 [kg]
param.m4 = mean(sol.parameters.m1.value); % mass of link 4 [kg]
param.m5 = mean(sol.parameters.m5.value); % mass of link 5 (trunk) [kg]

param.l1 = mean(sol.parameters.l1.value); % length of link n [m]
param.l2 = mean(sol.parameters.l2.value);
param.l3 = mean(sol.parameters.l2.value);
param.l4 = mean(sol.parameters.l1.value);
param.l5 = mean(sol.parameters.l5.value);

param.I1 = mean(sol.parameters.I1.value); % rotational inertia [kg m^2]
param.I2 = mean(sol.parameters.I2.value);
param.I3 = mean(sol.parameters.I2.value);
param.I4 = mean(sol.parameters.I1.value);
param.I5 = mean(sol.parameters.I5.value);

param.g = mean(sol.parameters.g.value); % gravitational acceleration [m/s^2]

simout = [sol.states.th1.value; sol.states.th2.value; sol.states.th3.value; sol.states.th4.value; sol.states.th5.value; sol.states.dth1.value; sol.states.dth2.value; sol.states.dth3.value; sol.states.dth4.value; sol.states.dth5.value]';
u = [sol.controls.u2.value; sol.controls.u3.value; sol.controls.u4.value; sol.controls.u5.value]';
time = sol.states.time.value;

x_F.th1 = simout(end,1);
x_F.th2 = simout(end,2);
x_F.th3 = simout(end,3);
x_F.th4 = simout(end,4);
x_F.th5 = simout(end,5);

x_I.th1 = simout(1,1);
x_I.th2 = simout(1,2);
x_I.th3 = simout(1,3);
x_I.th4 = simout(1,4);
x_I.th5 = simout(1,5);

th_I = [x_I.th1; x_I.th2; x_I.th3; x_I.th4; x_I.th5];
th_new = [(x_F.th1 + x_F.th2 + x_F.th3 + x_F.th4 - pi); (-x_F.th4);(-x_F.th3);(-x_F.th2);(x_F.th5 - x_F.th3 + pi)];

%% Calculating specific resistance

u_2_ext = [u(:,1); u(end,1)];
u_3_ext = [u(:,2); u(end,2)];
u_4_ext = [u(:,3); u(end,3)];
u_5_ext = [u(:,4); u(end,4)];

energy = trapz(time, abs(simout(:,7).*u_2_ext) + abs(simout(:,8).*u_3_ext) + abs(simout(:,9).*u_4_ext) + abs(simout(:,10).*u_5_ext));

% energy = 0;
% for i =1:12
%     energy = energy + 0.5*(power(i+1) + power(i))*(time(i+1) - time(i));
% end

avg_power = energy/time(end);
x1 = 0; % x position of the stance leg to calculate CoM x position
x_CoM_I = (param.m4*(x1 + (param.l4*cos(x_I.th1 + x_I.th2 + x_I.th3 + x_I.th4))/2 + param.l2*cos(x_I.th1 + x_I.th2) + param.l1*cos(x_I.th1) + param.l3*cos(x_I.th1 + x_I.th2 + x_I.th3)) + param.m1*(x1 + (param.l1*cos(x_I.th1))/2) + param.m3*(x1 + param.l2*cos(x_I.th1 + x_I.th2) + param.l1*cos(x_I.th1) + (param.l3*cos(x_I.th1 + x_I.th2 + x_I.th3))/2) + param.m5*(x1 + param.l2*cos(x_I.th1 + x_I.th2) + param.l1*cos(x_I.th1) + (param.l5*cos(x_I.th1 + x_I.th2 + x_I.th5))/2) + param.m2*(x1 + (param.l2*cos(x_I.th1 + x_I.th2))/2 + param.l1*cos(x_I.th1)))/(param.m1 + param.m2 + param.m3 + param.m4 + param.m5);
x_CoM_F = (param.m4*(x1 + (param.l4*cos(x_F.th1 + x_F.th2 + x_F.th3 + x_F.th4))/2 + param.l2*cos(x_F.th1 + x_F.th2) + param.l1*cos(x_F.th1) + param.l3*cos(x_F.th1 + x_F.th2 + x_F.th3)) + param.m1*(x1 + (param.l1*cos(x_F.th1))/2) + param.m3*(x1 + param.l2*cos(x_F.th1 + x_F.th2) + param.l1*cos(x_F.th1) + (param.l3*cos(x_F.th1 + x_F.th2 + x_F.th3))/2) + param.m5*(x1 + param.l2*cos(x_F.th1 + x_F.th2) + param.l1*cos(x_F.th1) + (param.l5*cos(x_F.th1 + x_F.th2 + x_F.th5))/2) + param.m2*(x1 + (param.l2*cos(x_F.th1 + x_F.th2))/2 + param.l1*cos(x_F.th1)))/(param.m1 + param.m2 + param.m3 + param.m4 + param.m5);
avg_vel =  (x_CoM_F - x_CoM_I)/time(end);
M_total = 2*param.m1 + 2*param.m2 + param.m5;
specific_resistance = avg_power/(M_total*param.g*avg_vel);
end

