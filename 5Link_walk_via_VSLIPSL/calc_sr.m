function [specific_resistance, avg_vel] = calc_sr(simout, inputTorque, flag, param, time)
%CALC_SR Function to calculate specific resistance from OpenOCL solution
% (modified for underactuated system)
%   Detailed explanation goes here

params = [param.m1; param.m2; param.m5; param.l1; param.l2; param.l5; param.g; param.I1; param.I2; param.I5; param.r_k; param.r_h; param.k_ba; param.phi_h0; param.phi_k0];

%% Calculating specific resistance

% pick starting and end point indices for one step
% start_pt = 99536;
% end_pt = 99905;

start_pt = 21269;
end_pt = 21636;

x_F.th1 = simout(end_pt,1);
x_F.th2 = simout(end_pt,2);
x_F.th3 = simout(end_pt,3);
x_F.th4 = simout(end_pt,4);
x_F.th5 = simout(end_pt,5);

x_I.th1 = simout(start_pt,1);
x_I.th2 = simout(start_pt,2);
x_I.th3 = simout(start_pt,3);
x_I.th4 = simout(start_pt,4);
x_I.th5 = simout(start_pt,5);

u_1_ext = inputTorque(start_pt:end_pt, 1);
u_2_ext = inputTorque(start_pt:end_pt, 2);
u_3_ext = inputTorque(start_pt:end_pt, 3);
u_4_ext = inputTorque(start_pt:end_pt, 4);
u_5_ext = inputTorque(start_pt:end_pt, 5);

energy = trapz(time(start_pt:end_pt), abs(simout(start_pt:end_pt,6).*u_1_ext) + abs(simout(start_pt:end_pt,7).*u_2_ext) + abs(simout(start_pt:end_pt,8).*u_3_ext) + abs(simout(start_pt:end_pt,9).*u_4_ext) + abs(simout(start_pt:end_pt,10).*u_5_ext));

% energy = 0;
% for i =1:12
%     energy = energy + 0.5*(power(i+1) + power(i))*(time(i+1) - time(i));
% end

avg_power = energy/(time(end_pt) - time(start_pt));
CoM_I = calculate_com(simout(start_pt,:), params, flag(start_pt,2), [0;0;0;0;0]);
CoM_F = calculate_com(simout(end_pt,:), params, flag(end_pt,2), [0;0;0;0;0]);
x_CoM_I = CoM_I(1);
x_CoM_F = CoM_F(1);
avg_vel =  (x_CoM_F - x_CoM_I)/(time(end_pt) - time(start_pt));
M_total = 2*param.m1 + 2*param.m2 + param.m5;
specific_resistance = avg_power/(M_total*param.g*avg_vel);

plot(inputTorque(start_pt:end_pt,1))
end

