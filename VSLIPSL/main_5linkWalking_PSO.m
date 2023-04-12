%% five_link_walking_sim
tic
clear all
close all

%% SLIP Data

% 2021.09.01: new SLIP-SL trajectory with low CoT (CoT = 0.1907)
load('SLIPSL_Data\dc_comp2021-09-01-16-06') % SLIPSLOpenOCL2021-09-01-15-36 (@C:\Matlab Workspace\SLIPSL_OpenOCL\202108111747 - SLIPSL_OpenOCL (True CoM))

dc = calc_slipsl_traj(dc); % function to calculate CoM traj. at SS phase (appends to dc)

slipsl.time_ss = dc.time_ss';
slipsl.x_ss = dc.ss_traj.x_CoM_ss;
slipsl.y_ss = dc.ss_traj.y_CoM_ss;
slipsl.dx_ss = dc.ss_traj.dx_CoM_ss;
slipsl.dy_ss = dc.ss_traj.dy_CoM_ss;

slipsl.time_ds = dc.time_ds';
slipsl.x_ds = dc.simout_ds(1,:)';
slipsl.y_ds = dc.simout_ds(2,:)';
slipsl.dx_ds = dc.simout_ds(3,:)';
slipsl.dy_ds = dc.simout_ds(4,:)';

swFootSLIPSL.x_CoM = slipsl.x_ss;
swFootSLIPSL.x = dc.ss_traj.x_sw;
swFootSLIPSL.z = dc.ss_traj.y_sw;
swFootSLIPSL.dx = dc.ss_traj.dx_sw;
swFootSLIPSL.dz = dc.ss_traj.dy_sw;

slipslParams = [
    dc.const_param.m_M; dc.const_param.m_swLeg; dc.const_param.m_swFoot; dc.const_param.L_thigh; dc.const_param.I_swLeg; dc.const_param.I_swFoot; dc.const_param.gravi;
    dc.col_param.k0_ss; dc.col_param.L0_ss; dc.col_param.k0_ds; dc.col_param.L0_ds; 
    dc.col_param.k_swFoot; dc.col_param.k_swLeg; dc.col_param.theta0; dc.col_param.r0; dc.col_param.foot; dc.col_param.footPlus];

% 2021.09.01: new SLIP-SL trajectory with low CoT (CoT = 0.1907)
load('SLIPSL_Data\ref_star2021-09-03-14-20') % dc_comp2021-09-01-16-06

ref_star_ss = [ref_star.ss.x_M_star, ... 
    ref_star.ss.y_M_star, ref_star.ss.x_swFoot_star, ref_star.ss.y_swFoot_star, ...
    ref_star.ss.del_y_M_star, ref_star.ss.del_x_swFoot_star, ref_star.ss.del_y_swFoot_star, ...
    ref_star.ss.del2_y_M_star, ref_star.ss.del2_x_swFoot_star, ref_star.ss.del2_y_swFoot_star, ...
    ref_star.ss.x_CoM_star];

ref_star_ds = [ref_star.ds.x_CoM_star, ...
    ref_star.ds.y_CoM_star, ref_star.ds.dx_CoM_star, ...
    ref_star.ds.del_y_CoM_star, ref_star.ds.del_dx_CoM_star, ...
    ref_star.ds.del2_y_CoM_star];

%%
sample_time = 0.001;

%% Model parameters

params.m1 = 4.75;
params.m2 = 5.25;

params.m5 = 60; % 5,1,1
params.m3 = params.m2;
params.m4 = params.m1;

params.l1 = 0.55;
params.l2 = 0.5;
params.l3 = params.l2;
params.l4 = params.l1;
params.l5 = 0.3;

params.g = 9.81;

params.I1 = params.m1*params.l1^2/12;
params.I2 = params.m2*params.l2^2/12;
params.I3 = params.I1;
params.I4 = params.I2;
params.I5 = params.m5*params.l5^2/12;

param = [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; params.I1; params.I2; params.I5];

%% Getting the ground reaction forces equation

slipslTraj_ss = [ref_star.dc.ss.time, ref_star.dc.ss.x_CoM', ref_star.dc.ss.y_CoM', ref_star.dc.ss.dx_CoM', ref_star.dc.ss.dy_CoM'];
slipslTraj_ds = [ref_star.dc.ds.time - ref_star.dc.ds.time(1), ref_star.dc.ds.x_CoM, ref_star.dc.ds.y_CoM, ref_star.dc.ds.dx_CoM, ref_star.dc.ds.dy_CoM];

swFootTraj = [ref_star.dc.ss.x_CoM', ref_star.dc.ss.x_sw', ref_star.dc.ss.y_sw', ref_star.dc.ss.dx_sw', ref_star.dc.ss.dy_sw']; % swing foot reference trajectory

%% Calculate Link Lengths
[q0, link_lengths] = calculate_optimal_link_lengths(dc, param)

%% Initial conditions
N = 1; % starting collocation point
% N = dc.N_ss + 1;
x_CoM_des = slipsl.x_ss(N);
z_CoM_des = slipsl.y_ss(N);
x_sw_des = swFootSLIPSL.x(N);
z_sw_des = swFootSLIPSL.z(N);
dx_CoM_des = slipsl.dx_ss(N);
dz_CoM_des = slipsl.dy_ss(N);
des_dx_sw = swFootSLIPSL.dx(N);
des_dz_sw = swFootSLIPSL.dz(N);
% [q0, dq0] = calculate_init_pos(x_CoM_des, z_CoM_des, x_sw_des, z_sw_des, dx_CoM_des, dz_CoM_des, des_dx_sw, des_dz_sw, param);
[q0, dq0] = calculate_init_pos_v2(x_CoM_des, z_CoM_des, x_sw_des, z_sw_des, dx_CoM_des, dz_CoM_des, des_dx_sw, des_dz_sw, param);
clear x_CoM_des z_CoM_des x_sw_des z_sw_des dx_CoM_des dz_CoM_des des_dx_sw des_dz_sw

X = [q0; dq0];

Initial_state = [q0;dq0];

%% init flag
close
% CoM = calculate_com(X, param, foot, ddq);
CoM = calculate_com(X, param, 0, [0;0;0;0;0]);
init_flag = [1; 0; -dc.col_param.footPlus; 0; CoM(1); CoM(2)];
clear CoM

%% VSLIPSL Controller gains and ref trajectories
% Controller Gains
gain_VSLIPSL.K_p = 200;
gain_VSLIPSL.K_d = 40;

gain_VSLIPSL.K_p_sw = 200;
gain_VSLIPSL.K_d_sw = 40;

gain_VSLIPSL.K_p_ds = 200;
gain_VSLIPSL.K_d_ds = 40;
gain_VSLIPSL.K_v_ds = 40; % 5 in visser 2012

gains_VSLIPSL = [gain_VSLIPSL.K_p; gain_VSLIPSL.K_d; gain_VSLIPSL.K_p_sw; gain_VSLIPSL.K_d_sw; gain_VSLIPSL.K_p_ds; gain_VSLIPSL.K_d_ds; gain_VSLIPSL.K_v_ds];

%% PSO

repeat_step = 1;
pNumber = 1; % number of particles
search_step = 1; % number of times to try the particles

f_reset = 1; % choose if you have reset best particle or not
f_record = 0; % choose whether pBestMemo should be saved automatically
f_dist = [0; -100; 0]; % flag for activating or deactivating the dist forces and choosing their magnitudes [on/off; F_dist_x, F_dist_y]

Tf = 15; % simulation finish time [secs]

for k = 1:repeat_step
    display(k)
    
    %% create initial particles
    % if I have reset the pBest file, I should set f_reset = 1
    [ptc, pBest, pBestPtc, vel] = init_ptc(pNumber, f_reset);
    load('pBest\pBestMemo.mat')
    f_reset = 0; %
    
    %% Do the loop
    
    for i = 1:search_step
        fprintf ('%.4f / %.4f', i, search_step)
        for j = 1:pNumber
            
            gain.kP1 = ptc(j,1); % gain for total energy controller
            gain.kP2 = ptc(j,2); % gain for total energy controller
            gain.kP3 = ptc(j,3); % gain for total energy controller
            gain.kP4 = ptc(j,4); % gain for total energy controller
            gain.kP5 = ptc(j,5); % gain for total energy controller
            
            gain.kD1 = ptc(j,6); % gain for total energy controller
            gain.kD2 = ptc(j,7); % gain for total energy controller
            gain.kD3 = ptc(j,8); % gain for total energy controller
            gain.kD4 = ptc(j,9); % gain for total energy controller
            gain.kD5 = ptc(j,10); % gain for total energy controller
            
            gains = [gain.kP1, gain.kD1, gain.kP2, gain.kD2, gain.kP3, gain.kD3, gain.kP4, gain.kD4, gain.kP5, gain.kD5]; % gains vector
            
            % Run the simulation
            gamma = 5;
            
            open_system('model_5LinkWalking')
            sim('model_5LinkWalking')
            
            % Cost of Transport
            dq = simout(:, 6:10); % joint velocities
            power = abs(dq.*inputTorque);
            energy = power.*sample_time;
            distance = CoM_acc(end,1); % distance that CoM traveled
            CoT = sum(sum(energy))/((params.m1 + params.m2 + params.m3 + params.m4 + params.m5)*params.g*distance);
            
            % trunk angle difference(pseudo limit cycle) ------------------
            % PSO awards for keeping the runk angle close to the avergae
            % value
            trunk_angle = simout(:,1) + simout(:,2) + simout(:,5);
            avgTrunk = mean(trunk_angle);
            trunkAngleDiff = trunk_angle - avgTrunk;
            totalTrunkAngleDiff = sum(abs(trunkAngleDiff*sample_time)); % area of difference between average trunk angle and trunk angle
            %--------------------------------------------------------------
            
            % Area under the distance between VLO points
            if time(end)>Tf -1
                theta_P(:,1) = rt_VLO(1,1,:); % atan2(dz_CoM, dx_CoM)
                z_P(:,1) = rt_VLO(1,2,:); % z_CoM position at VLO
                E_CoM(:,1) = rt_VLO(1,4,:); % energy of CoM at VLO
                %             VLOtimes = rt_VLO(1,5,:); % times when VLO happens
                
                vector_pc = [theta_P(2:end), z_P(2:end), E_CoM(2:end)] - [theta_P(1:end-1), z_P(1:end-1), E_CoM(1:end-1)]; % differences between consequent VLO points
                vector_pc = [vector_pc(:,1)/mean(theta_P), vector_pc(:,2)/mean(z_P), vector_pc(:,3)/mean(E_CoM)]; % normalizing these difference with average values
                vector_pc = abs(vector_pc);
                %             dist = sqrt(vector_pc(:,1).^2 + vector_pc(:,2).^2 + vector_pc(:,3).^2);
                
                VLO_diff = sum(sum([(vector_pc(1:end-1, :) + vector_pc(2:end, :))/2]));
                
                % another method to calculate
                mean_diff = sqrt(vector_pc(:,1).^2 + vector_pc(:,2).^2 + vector_pc(:,3).^2);
                trapz_VLO_diff = trapz(1:length(mean_diff), mean_diff);
                clear theta_P z_P E_CoM vector_pc
            end
            
            % -----------------adding constraints to gain values-----------
            % gain limits
            gainLimits = [
                0, 3000;
                0, 2000;
                0, 3000;
                0, 2000;
                0, 3000;
                0, 2000;
                0, 3000;
                0, 2000;
                0, 3000;
                0, 2000];
            
            overLimit = ptc(j,:)' - gainLimits(:,2);
            overLimit = max(0, overLimit);
            
            underLimit = min(0, ptc(j,:));
            
            if mean(overLimit) == 0 && mean(underLimit) == 0
                % if all gain values are under limit
                gainOverLim = 1;
            elseif mean(overLimit) == 0
                gainOverLim =sum(abs(underLimit));
            elseif mean(underLimit) == 0
                gainOverLim =sum(abs(overLimit));
            else
                % if at least 1 gain value is over limit
                gainOverLim = sum(overLimit)*sum(abs(underLimit));
            end
            
            % when -1<gainOverLim<1, it gets higher feasibility value
            % because feasibility is divided by this value.
            if gainOverLim < 1
                gainOverLim = 100; % set gainOverlim to a large
            end
            %--------------------------------------------------------------
            
            %%%%%% testing the ZMP condition %%%%%%%%%%%%%%
            
            % get the ground reaction forces
            % GRF(:,1) = F_x_GRF, GRF(:,2) = F_y_GRF
            [ddq_GRF, GRF] = calc_GRF(time, simout, param, flag, inputTorque);
            F_x = GRF(:,1); % Reaction forces at the ankle
            F_y = GRF(:,2);
            
            % Calculating ZMP
            m_foot = 1;
            x_foot_CoM = 0.01;
            F_y_GRF = -(-m_foot*params.g + -F_y);
            x_ZMP = (-inputTorque(:,1) + x_foot_CoM*m_foot*params.g)./F_y_GRF;
            
            ZMP_fitness = max(abs(x_ZMP(10/sample_time:end)));
            %============================================================%
            
            %%%%%% Adding tracking error term to the fitness value %%%%%%
            y_err = des_z_dz_dx(:,2) - CoM_acc(:,2);
            y_err = sum(abs(y_err(10/sample_time:end)));
            
            %dx_avg = mean(CoM_acc(10/sample_time:end, 3));
            %dx_avg_slipsl = (dc.x_CoM(end) - dc.x_CoM(1))/dc.time(end);
            %dx_err = abs(dx_avg_slipsl - dx_avg);
            
            dx_err = (des_z_dz_dx(:,3) - CoM_acc(:,3));
            dx_err = sum(abs(dx_err(10/sample_time:end)));
            %=============================================================%
            
            if time(end) >= Tf - 1
                % fitness(j) = time(end)/(CoT) + time(end)/(CoT*totalTrunkAngleDiff*VLO_diff);
                % fitness(j) = (time(end)+time(end)/(VLO_diff*max(abs(x_ZMP(10/sample_time:end)))))/(gainOverLim);
                % fitness(j) = (time(end)+time(end)/(VLO_diff*ZMP_fitness*y_err*dx_err))/(gainOverLim);
                % fitness(j) = (time(end)+time(end)/(trapz_VLO_diff*max(0.3, ZMP_fitness)*CoT))/(gainOverLim);
                fitness(j) = (time(end)+time(end)/(max(1, trapz_VLO_diff)*max(0.3, ZMP_fitness)))/(gainOverLim);
            else
                fitness(j) = (time(end))/gainOverLim;
                VLO_diff = 0; % if robot falls quickly
                ZMP_fitness = 0;
                y_err = 0;
                dx_err = 0;
            end
            
            display_str = ['PTC #: ', num2str(j), '/', num2str(pNumber), ', Fitness Value: ', num2str(fitness(j)),  ', Search Step: ', num2str(i), '/', num2str(search_step), ', Repeat Step: ', num2str(k), '/', num2str(repeat_step), ', pBest: ', num2str(pBest)] ;
            display(display_str)
            
            if fitness(j) > pBest
                pBest = fitness(j);
                pBestPtc = ptc(j,:); % updating the best particle
                %                  pBestPtcHist = [double(pBest), pBestPtc, step_no(end),CoT];
                pBestPtcHist = [double(pBest), pBestPtc, time(end), CoT, totalTrunkAngleDiff, VLO_diff, gainOverLim, ZMP_fitness, y_err, dx_err];
                sz = size(pBestMemo);
                pBestMemo(sz(1)+1, :) = pBestPtcHist;
                save('pBest\pBestMemo', 'pBestMemo')
                
                display(['Pbest: ', num2str(pBest)]);
            end
            
            % check which particle performed the best
            if j == 1 % set gBest to fitness value of the first particle
                gBest = fitness(j);
                gBestPtc = ptc(j,:); % updating the best particle
            end
            
            if fitness(j) > gBest
                gBest = fitness(j);
                gBestPtc = ptc(j,:); % updating the best particle
            end
        end
        
        % Calculate the particle velocities and updating them
        for j = 1:pNumber
            % calculate velocity
            c1 = 2.05;
            c2 = c1;
%             consFactor = 2/(2 - (c1 + c2) - sqrt((c1 + c2)^2+4*(c1 + c2))); % from paper called "A simple and efficient constrained particle swarm optimization and its application to engineering design problems"
%             vel(j,:) = consFactor*(vel(j,:) + c1*rand()*(pBestPtc - ptc(j,:)) + c2*rand()*(gBestPtc - ptc(j,:)));
            vel(j,:) = vel(j,:) + c1*rand()*(pBestPtc - ptc(j,:)) + c2*rand()*(gBestPtc - ptc(j,:));
            
            velLim = 100;
            vel(j,:) = min(velLim, vel(j,:));
            vel(j,:) = max(-velLim, vel(j,:));
            
            % update the particles
            ptc(j,:) = vel(j,:) + ptc(j,:);
        end
    end
    
    %%
    
    if f_record == 1
        filename = sprintf('pBestMemo%s.mat', datestr(now,'yyyy-mm-dd-HH-MM'));
        subfolder = 'pBest';
        save(fullfile(subfolder,filename),'pBestMemo')
    end
    
end

%% Stability Analysis
% figure_poincare(simout, param, flag(:,2), step_no, rt_VLO, time)

%% CoM acceleration comparison
% trackingPlots(simout, des_z_dz_dx, CoM_acc, sw_ft_pos, sw_ft_des, flag, time)
trackingPlots_jointAngles(simout, flag, time, des_th)

%% Animation
f_animation = 1;
if f_animation == 1
    f_video = 1; % flag for recrding video
    f_pause = 0;
    frame_leap = 20;
    % animation(f_video, simout, sample_time, sw_ft_des, param, f_pause, frame_leap, flag, step_no, force, des_z_dz_dx)
    animation(f_video, simout, sample_time, sw_ft_des, param, f_pause, frame_leap, flag, step_no, force, des_traj, slipslParams, des_th)
end
%% Calculating ZMP

% get the ground reaction forces
% GRF(:,1) = F_x_GRF, GRF(:,2) = F_y_GRF
[ddq_GRF, GRF, U_test] = calc_GRF(time, simout, param, flag, inputTorque);
lambda_GRF_x = GRF(:,1); % Reaction forces at the ankle
lambda_GRF_y = GRF(:,2);

m_foot = 2;
x_foot_CoM = 0.1;
F_y_GRF = -(-m_foot*params.g + -lambda_GRF_y);
x_ZMP = (inputTorque(:,1) + x_foot_CoM*m_foot*params.g)./F_y_GRF;

% x_ZMP_new = (inputTorque(:,1) + x_foot_CoM*m_foot*params.g) ./ (F_y_GRF - m_foot*params.g);

medfiltX_ZMP = medfilt1(x_ZMP,20);

figure()
plot(time, medfiltX_ZMP)
% plot(time, x_ZMP)
hold on

plot(time, 0.3*flag(:,1))
grid on
ylabel('x_{ZMP} [m]')
xlim([5,15])

max_x_ZMP = max(medfiltX_ZMP(5/sample_time:end))
min_x_ZMP = min(medfiltX_ZMP(5/sample_time:end))

% plot(time, x_ZMP_new)
% plot(time, medfiltLoopVoltage)
toc

%% encoder-decoder elapsed time

encoder_time_elapsed = encoder_time_elapsed(~isnan(encoder_time_elapsed));
mean_encoder__time = mean(encoder_time_elapsed)
max_encoder__time = max(encoder_time_elapsed)

mean_decoder_time = mean(decoder_time_elapsed)
max_decoder_time_elapsed = max(decoder_time_elapsed)


%%

function dc = calc_slipsl_traj(dc)
% constant parameters
m_M = dc.const_param.m_M;
m_swLeg = dc.const_param.m_swLeg;
m_swFoot = dc.const_param.m_swFoot;
L_thigh = dc.const_param.L_thigh;
I_swLeg = dc.const_param.I_swLeg;
I_swFoot = dc.const_param.I_swFoot;
gravi = dc.const_param.gravi;

x_M = dc.simout_ss(1,:)';
y_M = dc.simout_ss(2,:)';
theta = dc.simout_ss(3,:)';
r = dc.simout_ss(4,:)';

dx_M = dc.simout_ss(5,:)';
dy_M = dc.simout_ss(6,:)';
dtheta = dc.simout_ss(7,:)';
dr = dc.simout_ss(8,:)';

dc.ss_traj.x_CoM_ss = (m_swLeg*(x_M + (L_thigh*cos(theta))/2) + m_M*x_M + m_swFoot*(x_M + cos(theta).*(L_thigh + r)))/(m_M + m_swLeg + m_swFoot);
dc.ss_traj.y_CoM_ss = (m_swFoot*(y_M + sin(theta).*(L_thigh + r)) + m_swLeg*(y_M + (L_thigh*sin(theta))/2) + m_M*y_M)/(m_M + m_swLeg + m_swFoot);

dc.ss_traj.x_sw = x_M + cos(theta).*(L_thigh + r);
dc.ss_traj.y_sw = y_M + sin(theta).*(L_thigh + r);

dc.ss_traj.dx_CoM_ss = dx_M + (dr.*m_swFoot.*cos(theta))/(m_M + m_swLeg + m_swFoot) - (dtheta.*sin(theta).*((L_thigh*m_swLeg)/2 + L_thigh*m_swFoot + m_swFoot*r))/(m_M + m_swLeg + m_swFoot);
dc.ss_traj.dy_CoM_ss = dy_M + (dr.*m_swFoot.*sin(theta))/(m_M + m_swLeg + m_swFoot) + (dtheta.*cos(theta).*((L_thigh*m_swLeg)/2 + L_thigh*m_swFoot + m_swFoot*r))/(m_M + m_swLeg + m_swFoot);

dc.ss_traj.dx_sw = dx_M + dr.*cos(theta) - dtheta.*sin(theta).*(L_thigh + r);
dc.ss_traj.dy_sw = dy_M + dr.*sin(theta) + dtheta.*cos(theta).*(L_thigh + r);
end