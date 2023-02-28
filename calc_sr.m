function [specific_resistance, avg_vel] = calc_sr(simout, inputTorque, flag, params, time)
%CALC_SR Function to calculate specific resistance from simulation
% (modified for underactuated system)
%   Detailed explanation goes here

param = [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; params.I1; params.I2; params.I5; params.r_k; params.r_h; params.k_ba; params.phi_h0; params.phi_k0];

%% Determining one step
% f_impact_happened = -1;
% impact_indexes = [];
% for i =1:length(time)
%     f_impact = flag(i,1);
%     if f_impact == 1 && f_impact_happened == -1
%         f_impact_happened = 1;
%         impact_indexes = [impact_indexes;i];
%     elseif f_impact == -1 && f_impact_happened == 1
%         f_impact_happened = -1;
%     end
%     
% end

%% Calculating specific resistance

% pick starting and end point indices for one step
start_pt = 1;
end_pt = length(time);

% no_of_impacts = length(impact_indexes);
% start_pt = impact_indexes(no_of_impacts-3);
% end_pt = impact_indexes(no_of_impacts-2)-1;

u_1_ext = inputTorque(start_pt:end_pt, 1); % u_1 is set as zero so no need to delete this (copied from fully actuated model)
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
CoM_I = calculate_com(simout(start_pt,:), param, flag(start_pt,:), [0;0;0;0;0]);
CoM_F = calculate_com(simout(end_pt,:), param, flag(end_pt,:), [0;0;0;0;0]);
x_CoM_I = CoM_I(1);
x_CoM_F = CoM_F(1);
avg_vel =  (x_CoM_F - x_CoM_I)/(time(end_pt) - time(start_pt));
M_total = 2*params.m1 + 2*params.m2 + params.m5;
specific_resistance = avg_power/(M_total*params.g*avg_vel);

% plot(inputTorque(start_pt:end_pt,1))
end

