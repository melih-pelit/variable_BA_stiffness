function [CoT, avg_vel] = calc_CoT(simout, inputTorque, flag, params, time)
%CALC_SR Function to calculate specific resistance from OpenOCL solution
% (modified for underactuated system)
%   Detailed explanation goes here

param = [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; params.I1; params.I2; params.I5];

%% finding the starting point of the step
flag_prev = flag(1,1); % initializing flag prev
k = 1;
for i = 1:length(time)
    if flag(i,1) ~= flag_prev
        if flag(i,1) == 1
            rec_LO_idx(k) = i;
            k = k+1;
        else
            rec_TD_idx(k) = i;
        end
        
        flag_prev = flag(i,1);
    end
    
end

%% Calculating specific resistance

% pick starting and end point indices for one step
% start_pt = 99536;
% end_pt = 99905;

start_pt = rec_LO_idx(end-3);
end_pt = rec_LO_idx(end-2);

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
CoM_I = calculate_com(simout(start_pt,:), param, flag(start_pt,2), [0;0;0;0;0]);
CoM_F = calculate_com(simout(end_pt,:), param, flag(end_pt,2), [0;0;0;0;0]);
x_CoM_I = CoM_I(1);
x_CoM_F = CoM_F(1);
avg_vel =  (x_CoM_F - x_CoM_I)/(time(end_pt) - time(start_pt));
M_total = 2*params.m1 + 2*params.m2 + params.m5;
CoT = energy/(M_total*params.g*(x_CoM_F - x_CoM_I));

% plot(inputTorque(start_pt:end_pt,1))
end

