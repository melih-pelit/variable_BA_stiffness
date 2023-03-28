% 2022.03.09 5LinkWalk_OpenOCL.m
% Generating optimum inputs and walking trajectories for 5 Link 
% Bipedal Model using OpenOCL (4 input torques)
% Mustafa Melih PELIT

% clc
clear all

singleStance = ocl.Stage([], 'vars', @singleStance_vars, 'dae', @singleStance_ode, 'gridconstraints', @ss_gridconstraints, 'pathcosts', @ss_pathcosts, 'N', 12, 'd', 3);
doubleStance = ocl.Stage([], 'vars', @doubleStance_vars, 'dae', @doubleStance_ode, 'gridconstraints', @ds_gridconstraints, 'pathcosts', @ds_pathcosts, 'N', 12, 'd', 3);
ocp = ocl.MultiStageProblem({singleStance, doubleStance}, {@stage_transition},...
    'transition_type', 2);

%%%%% Constant Parameters %%%%%

param.m1 = 5; % mass of link 1 [kg]
param.m2 = 5; % mass of link 2 [kg]
param.m3 = param.m2; % mass of link 3 [kg]
param.m4 = param.m1; % mass of link 4 [kg]
param.m5 = 60; % mass of link 5 (trunk) [kg]

param.l1 = 0.48; % length of link n [m]
param.l2 = 0.48;
param.l3 = param.l2;
param.l4 = param.l1;
param.l5 = 0.48;

param.I1 = (param.m1*param.l1^2)/12; % rotational inertia [kg m^2]
param.I2 = (param.m2*param.l2^2)/12;
param.I3 = param.I2;
param.I4 = param.I1;
param.I5 = (param.m5*param.l5^2)/12;

param.g = 9.81; % gravitational acceleration [m/s^2]

param.d_obs = 0; % [m] distance of the obstacle from the stance foot
param.w_obs = 0.1;
param.h_obs = 0.02;

% setting OpenOCL parameters
singleStance.setBounds('m1'   , param.m1);
singleStance.setBounds('m2'   , param.m2);
singleStance.setBounds('m5'   , param.m5);

singleStance.setBounds('l1'   , param.l1);
singleStance.setBounds('l2'   , param.l2);
singleStance.setBounds('l5'   , param.l5);

singleStance.setBounds('I1'   , param.I1);
singleStance.setBounds('I2'   , param.I2);
singleStance.setBounds('I5'   , param.I5);

singleStance.setBounds('g'   , param.g);

singleStance.setBounds('d_obs'   , param.d_obs); % x location of the ellipse
singleStance.setBounds('w_obs'   , param.w_obs); % width of the ellipse
singleStance.setBounds('h_obs'   , param.h_obs); % height of the ellipse

% Double stance
doubleStance.setBounds('m1'   , param.m1);
doubleStance.setBounds('m2'   , param.m2);
doubleStance.setBounds('m5'   , param.m5);

doubleStance.setBounds('l1'   , param.l1);
doubleStance.setBounds('l2'   , param.l2);
doubleStance.setBounds('l5'   , param.l5);

doubleStance.setBounds('I1'   , param.I1);
doubleStance.setBounds('I2'   , param.I2);
doubleStance.setBounds('I5'   , param.I5);

doubleStance.setBounds('g'   , param.g);

%%%%% Collocation Parameters %%%%%
singleStance.setInitialBounds( 'time',   0.0);

% ocp.setInitialBounds( 'th3',   0, 2*pi);
% ocp.setInitialBounds( 'th5',   -pi, pi);

singleStance.setBounds('x_hip_I', -2, 2); % initial x position of the swing leg

% parameters for recording initial and final states
singleStance.setBounds('th1_I', -2*pi, 2*pi);
singleStance.setBounds('th2_I', -2*pi, 2*pi);
singleStance.setBounds('th3_I', -2*pi, 2*pi);
singleStance.setBounds('th4_I', -2*pi, 2*pi);
singleStance.setBounds('th5_I', -2*pi, 2*pi);

singleStance.setBounds('dth1_I', -10, 10);
singleStance.setBounds('dth2_I', -10, 10);
singleStance.setBounds('dth3_I', -10, 10);
singleStance.setBounds('dth4_I', -10, 10);
singleStance.setBounds('dth5_I', -10, 10);

singleStance.setBounds('x_CoM_I', -1, 1); % recording initial x_CoM position
singleStance.setBounds('x_CoM_F', -1, 1); % recording final x_CoM position
singleStance.setBounds('time_F', 0, 2); % recording the final time

%% Solve

sol = ocp.getInitialGuess();

% %%% using previous solutions
% ocp.getInitialGuess(); % this line is necessary to use old trajectories as initial guess

% loading previous results 
% load('results\5LinkWalkingOpenOCL2022-03-10-16-47') % result with v.o. disabled

[sol,times] = ocp.solve(sol);
% sr(1) = calc_sr(sol);

% simout = [sol.states.th1.value; sol.states.th2.value; sol.states.th3.value; sol.states.th4.value; sol.states.th5.value; sol.states.dth1.value; sol.states.dth2.value; sol.states.dth3.value; sol.states.dth4.value; sol.states.dth5.value]';
% u = [sol.controls.u2.value; sol.controls.u3.value; sol.controls.u4.value; sol.controls.u5.value]';
% time = sol.states.time.value;

j = 2;
%%
rpt = 1; % how many times to repeat?
for i=j:j+(rpt-1)
    [sol,times] = ocp.solve(sol);
%     sr(j) = calc_sr(sol)
    
%     simout = [sol.states.th1.value; sol.states.th2.value; sol.states.th3.value; sol.states.th4.value; sol.states.th5.value; sol.states.dth1.value; sol.states.dth2.value; sol.states.dth3.value; sol.states.dth4.value; sol.states.dth5.value]';
%     u = [sol.controls.u2.value; sol.controls.u3.value; sol.controls.u4.value; sol.controls.u5.value]';
%     time = sol.states.time.value;
    
    j = j+1;
end
%% Recording the solution
f_record = 1;
if f_record == 1
    filename = sprintf('5LinkWalkingOpenOCL%s.mat', datestr(now,'yyyy-mm-dd-HH-MM'));
    subfolder = 'results';
    save(fullfile(subfolder,filename),'sol')
end

%% Animation

f_video = 0;
f_pause = 0;

simout = [sol.states.th1.value; sol.states.th2.value; sol.states.th3.value; sol.states.th4.value; sol.states.th5.value; sol.states.dth1.value; sol.states.dth2.value; sol.states.dth3.value; sol.states.dth4.value; sol.states.dth5.value]';
u = [sol.controls.u2.value; sol.controls.u3.value; sol.controls.u4.value; sol.controls.u5.value]';
time = sol.states.time.value;

animation(f_video, simout, time, param, f_pause)

%% figures
% figures1(simout, des_z_dz_dx, CoM_acc, sw_ft_pos, sw_ft_des, flag, time)

%% Checking cyclic walking
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

cyclic_err = [
    wrapTo2Pi(x_F.th1 + x_F.th2 + x_F.th3 + x_F.th4 - pi) - wrapTo2Pi(x_I.th1);
    wrapTo2Pi(-x_F.th4) - wrapTo2Pi(x_I.th2);
    wrapTo2Pi(-x_F.th3) - wrapTo2Pi(x_I.th3);
    wrapTo2Pi(-x_F.th2) - wrapTo2Pi(x_I.th4);
    wrapTo2Pi(x_F.th5 - x_F.th3 + pi) - wrapTo2Pi(x_I.th5)];

x_sw = param.l1*cos(simout(:,1)) + param.l2*cos(simout(:,1) + simout(:,2)) + param.l2*cos(simout(:,1)+simout(:,2)+simout(:,3)) + param.l1*cos(simout(:,1)+simout(:,2)+simout(:,3)+simout(:,4));
y_sw = param.l1*sin(simout(:,1)) + param.l2*sin(simout(:,1) + simout(:,2)) + param.l2*sin(simout(:,1)+simout(:,2)+simout(:,3)) + param.l1*sin(simout(:,1)+simout(:,2)+simout(:,3)+simout(:,4));

dx_sw = -(param.l1.*sin(simout(:,1)).*(simout(:,6)) + param.l2.*sin(simout(:,1) + simout(:,2)).*(simout(:,6)+simout(:,7)) + param.l2.*sin(simout(:,1)+simout(:,2)+simout(:,3)).*(simout(:,6)+simout(:,7)+simout(:,8)) + param.l1.*sin(simout(:,1)+simout(:,2)+simout(:,3)+simout(:,4)).*(simout(:,6)+simout(:,7)+simout(:,8)+simout(:,9)));
dy_sw = param.l1.*cos(simout(:,1)).*(simout(:,6)) + param.l2.*cos(simout(:,1) + simout(:,2)).*(simout(:,6)+simout(:,7)) + param.l2.*cos(simout(:,1)+simout(:,2)+simout(:,3)).*(simout(:,6)+simout(:,7)+simout(:,8)) + param.l1.*cos(simout(:,1)+simout(:,2)+simout(:,3)+simout(:,4)).*(simout(:,6)+simout(:,7)+simout(:,8)+simout(:,9));

% drawing foot trajectory and virtual obstacle
figure 
plot(x_sw, y_sw)
hold on
% rectangle('Position',[param.d_obs - param.r_obs, 0- param.r_obs, 2*param.r_obs, 2*param.r_obs],'Curvature',[1 1], 'FaceColor', 'r')
plot_ellipse(param.w_obs, param.h_obs, param.d_obs, 0, 0, 'r')
plot(x_sw, y_sw, 'k.', 'MarkerSize', 10)
ylim([0,inf])
hold off

%% Calculating specific resistance
specific_resistance = calc_sr(sol);

%%

function singleStance_vars(sh)
% State = [th1; th2; th3; th4; th5; dth1; dth2; dth3; dth4; dth5]
sh.addState('th1', 'lb', 0, 'ub', pi); % add a state with lower and upper bounds
sh.addState('th2', 'lb', deg2rad(5), 'ub', deg2rad(22.5));
sh.addState('th3', 'lb', -2*pi, 'ub', 0);
sh.addState('th4', 'lb', deg2rad(270), 'ub', deg2rad(360));
sh.addState('th5', 'lb', -pi, 'ub', pi);

sh.addState('dth1', 'lb', -10, 'ub', 10);
sh.addState('dth2', 'lb', -10, 'ub', 10);
sh.addState('dth3', 'lb', -10, 'ub', 10);
sh.addState('dth4', 'lb', -10, 'ub', 10);
sh.addState('dth5', 'lb', -10, 'ub', 10);

% adding the control as a state so that GRF can be calculated in gridconstrains
% sh.addState('u1');  % joint 1 torque
sh.addControl('u2');  % 
sh.addControl('u3');  % 
sh.addControl('u4');  % 
sh.addControl('u5');  % 

sh.addState('time', 'lb', 0, 'ub', 2);

% % adding the time derivative of the control as the control
% % sh.addControl('du1');  % time derivative of joint 1 torque
% sh.addControl('du2');  % 
% sh.addControl('du3');  % 
% sh.addControl('du4');  % 
% sh.addControl('du5');  % 

sh.addParameter('m1');
sh.addParameter('m2');
sh.addParameter('m5');

sh.addParameter('l1');
sh.addParameter('l2');
sh.addParameter('l5');

sh.addParameter('I1');
sh.addParameter('I2');
sh.addParameter('I5');

sh.addParameter('g');

sh.addParameter('d_obs');
% sh.addParameter('r_obs');
sh.addParameter('h_obs');
sh.addParameter('w_obs');

% Collocation Parameters
sh.addParameter('x_hip_I');

sh.addParameter('th1_I');
sh.addParameter('th2_I');
sh.addParameter('th3_I');
sh.addParameter('th4_I');
sh.addParameter('th5_I');

sh.addParameter('dth1_I');
sh.addParameter('dth2_I');
sh.addParameter('dth3_I');
sh.addParameter('dth4_I');
sh.addParameter('dth5_I');

sh.addParameter('x_CoM_I');
sh.addParameter('x_CoM_F');
sh.addParameter('time_F');
end

function singleStance_ode(sh,x,~,u,p)

% States
th1 = x.th1;
th2 = x.th2;
th3 = x.th3;
th4 = x.th4;
th5 = x.th5;

dth1 = x.dth1;
dth2 = x.dth2;
dth3 = x.dth3;
dth4 = x.dth4;
dth5 = x.dth5;

%%%%% Parameters for dynamics %%%%%
m1 = p.m1; % mass of link 1 [kg]
m2 = p.m2; % mass of link 2 [kg]
m3 = p.m2; % mass of link 3 [kg]
m4 = p.m1; % mass of link 4 [kg]
m5 = p.m5; % mass of link 5 (trunk) [kg]

l1 = p.l1; % length of link n [m]
l2 = p.l2;
l3 = p.l2;
l4 = p.l1;
l5 = p.l5;

I1 = p.I1; % rotational inertia [kg m^2]
I2 = p.I2;
I3 = p.I2;
I4 = p.I1;
I5 = p.I5;

gravi = p.g; % gravitational acceleration [m/s^2]

% Inertia Matrix
MM = [
    I1 + I2 + I3 + I4 + I5 + (l1^2*m1)/4 + l1^2*m2 + l1^2*m3 + (l2^2*m2)/4 + l1^2*m4 + l2^2*m3 + l1^2*m5 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l1*l3*m3*cos(th2 + th3) + 2*l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + l1*l5*m5*cos(th2 + th5) + l1*l2*m2*cos(th2) + 2*l1*l2*m3*cos(th2) + 2*l1*l2*m4*cos(th2) + 2*l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + l1*l4*m4*cos(th2 + th3 + th4), I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2, I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2, I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2, I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2;
    I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                               I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l2*l4*m4*cos(th3 + th4) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5),                                                                                             I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4),                                     I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2,                               (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5;
    I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                                                                                                                 I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4),                                                                                                                                                                       I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + l3*l4*m4*cos(th4),                                                                   (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4,                                                                      0;
    I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                                                                                                                                                                                      I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2,                                                                                                                                                                                                (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4,                                                                                           (m4*l4^2)/4 + I4,                                                                      0;
    I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2,                                                                                                                                                                                                                                                                                                                                                                                                    (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5,                                                                                                                                                                                                                                       0,                                                                                                          0,                                                       (m5*l5^2)/4 + I5];


% Centrifugal force and Coriolis Force Vector
CC =  [
 - (dth2^2*l1*l3*m3*sin(th2 + th3))/2 - dth2^2*l1*l3*m4*sin(th2 + th3) - (dth3^2*l1*l3*m3*sin(th2 + th3))/2 - dth3^2*l1*l3*m4*sin(th2 + th3) - (dth2^2*l1*l5*m5*sin(th2 + th5))/2 - (dth3^2*l2*l4*m4*sin(th3 + th4))/2 - (dth4^2*l2*l4*m4*sin(th3 + th4))/2 - (dth5^2*l1*l5*m5*sin(th2 + th5))/2 - (dth2^2*l1*l2*m2*sin(th2))/2 - dth2^2*l1*l2*m3*sin(th2) - dth2^2*l1*l2*m4*sin(th2) - dth2^2*l1*l2*m5*sin(th2) - (dth3^2*l2*l3*m3*sin(th3))/2 - dth3^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 - (dth5^2*l2*l5*m5*sin(th5))/2 - (dth2^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - (dth3^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - (dth4^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - dth1*dth2*l1*l3*m3*sin(th2 + th3) - 2*dth1*dth2*l1*l3*m4*sin(th2 + th3) - dth1*dth3*l1*l3*m3*sin(th2 + th3) - 2*dth1*dth3*l1*l3*m4*sin(th2 + th3) - dth2*dth3*l1*l3*m3*sin(th2 + th3) - 2*dth2*dth3*l1*l3*m4*sin(th2 + th3) - dth1*dth2*l1*l5*m5*sin(th2 + th5) - dth1*dth3*l2*l4*m4*sin(th3 + th4) - dth1*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth3*l2*l4*m4*sin(th3 + th4) - dth2*dth4*l2*l4*m4*sin(th3 + th4) - dth1*dth5*l1*l5*m5*sin(th2 + th5) - dth3*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth5*l1*l5*m5*sin(th2 + th5) - dth1*dth2*l1*l2*m2*sin(th2) - 2*dth1*dth2*l1*l2*m3*sin(th2) - 2*dth1*dth2*l1*l2*m4*sin(th2) - 2*dth1*dth2*l1*l2*m5*sin(th2) - dth1*dth3*l2*l3*m3*sin(th3) - 2*dth1*dth3*l2*l3*m4*sin(th3) - dth2*dth3*l2*l3*m3*sin(th3) - 2*dth2*dth3*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4) - dth1*dth5*l2*l5*m5*sin(th5) - dth2*dth5*l2*l5*m5*sin(th5) - dth1*dth2*l1*l4*m4*sin(th2 + th3 + th4) - dth1*dth3*l1*l4*m4*sin(th2 + th3 + th4) - dth1*dth4*l1*l4*m4*sin(th2 + th3 + th4) - dth2*dth3*l1*l4*m4*sin(th2 + th3 + th4) - dth2*dth4*l1*l4*m4*sin(th2 + th3 + th4) - dth3*dth4*l1*l4*m4*sin(th2 + th3 + th4);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        (dth1^2*l1*l3*m3*sin(th2 + th3))/2 + dth1^2*l1*l3*m4*sin(th2 + th3) + (dth1^2*l1*l5*m5*sin(th2 + th5))/2 - (dth3^2*l2*l4*m4*sin(th3 + th4))/2 - (dth4^2*l2*l4*m4*sin(th3 + th4))/2 + (dth1^2*l1*l2*m2*sin(th2))/2 + dth1^2*l1*l2*m3*sin(th2) + dth1^2*l1*l2*m4*sin(th2) + dth1^2*l1*l2*m5*sin(th2) - (dth3^2*l2*l3*m3*sin(th3))/2 - dth3^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 - (dth5^2*l2*l5*m5*sin(th5))/2 + (dth1^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - dth1*dth3*l2*l4*m4*sin(th3 + th4) - dth1*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth3*l2*l4*m4*sin(th3 + th4) - dth2*dth4*l2*l4*m4*sin(th3 + th4) - dth3*dth4*l2*l4*m4*sin(th3 + th4) - dth1*dth3*l2*l3*m3*sin(th3) - 2*dth1*dth3*l2*l3*m4*sin(th3) - dth2*dth3*l2*l3*m3*sin(th3) - 2*dth2*dth3*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4) - dth1*dth5*l2*l5*m5*sin(th5) - dth2*dth5*l2*l5*m5*sin(th5);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            (dth1^2*l1*l3*m3*sin(th2 + th3))/2 + dth1^2*l1*l3*m4*sin(th2 + th3) + (dth1^2*l2*l4*m4*sin(th3 + th4))/2 + (dth2^2*l2*l4*m4*sin(th3 + th4))/2 + (dth1^2*l2*l3*m3*sin(th3))/2 + dth1^2*l2*l3*m4*sin(th3) + (dth2^2*l2*l3*m3*sin(th3))/2 + dth2^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 + (dth1^2*l1*l4*m4*sin(th2 + th3 + th4))/2 + dth1*dth2*l2*l4*m4*sin(th3 + th4) + dth1*dth2*l2*l3*m3*sin(th3) + 2*dth1*dth2*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      (l4*m4*(dth1^2*l2*sin(th3 + th4) + dth2^2*l2*sin(th3 + th4) + dth1^2*l3*sin(th4) + dth2^2*l3*sin(th4) + dth3^2*l3*sin(th4) + dth1^2*l1*sin(th2 + th3 + th4) + 2*dth1*dth2*l2*sin(th3 + th4) + 2*dth1*dth2*l3*sin(th4) + 2*dth1*dth3*l3*sin(th4) + 2*dth2*dth3*l3*sin(th4)))/2;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           (l5*m5*(dth1^2*l1*sin(th2 + th5) + dth1^2*l2*sin(th5) + dth2^2*l2*sin(th5) + 2*dth1*dth2*l2*sin(th5)))/2];

% Gravity Vector
GG = [
    
 
 (m3*(l2*cos(th1 + th2) + l1*cos(th1) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + l1*cos(th1) + (l5*cos(th1 + th2 + th5))/2) + m2*((l2*cos(th1 + th2))/2 + l1*cos(th1)) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3)) + (l1*m1*cos(th1))/2)*gravi;
                                                                                (m3*(l2*cos(th1 + th2) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + (l5*cos(th1 + th2 + th5))/2) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) + (l2*m2*cos(th1 + th2))/2)*gravi;
                                                                                                                                                                                                            (m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l3*cos(th1 + th2 + th3)) + (l3*m3*cos(th1 + th2 + th3))/2)*gravi;
                                                                                                                                                                                                                                                                         ((l4*m4*cos(th1 + th2 + th3 + th4))/2)*gravi;
                                                                                                                                                                                                                                                                               ((l5*m5*cos(th1 + th2 + th5))/2)*gravi];
% calculating the accelerations
% tau = [x.u1; x.u2; x.u3; x.u4; x.u5 - x.u3];
tau = [0; u.u2; u.u3; u.u4; u.u5 - u.u3];
ddq = inv(MM)*(tau - CC - GG);

sh.setODE('th1', x.dth1);
sh.setODE('th2', x.dth2);
sh.setODE('th3', x.dth3);
sh.setODE('th4', x.dth4);
sh.setODE('th5', x.dth5);

sh.setODE('dth1', ddq(1) );
sh.setODE('dth2', ddq(2) );
sh.setODE('dth3', ddq(3) );
sh.setODE('dth4', ddq(4) );
sh.setODE('dth5', ddq(5) );

% sh.setODE('u2', u.du2 );
% sh.setODE('u3', u.du3 );
% sh.setODE('u4', u.du4 );
% sh.setODE('u5', u.du5 );

sh.setODE('time', 1); % why this is set to 1, not sure. probably this increases t as much as the increment
end

function ss_gridconstraints(ch, k, K, x, p)

% States
th1 = x.th1;
th2 = x.th2;
th3 = x.th3;
th4 = x.th4;
th5 = x.th5;

dth1 = x.dth1;
dth2 = x.dth2;
dth3 = x.dth3;
dth4 = x.dth4;
dth5 = x.dth5;

% Parameters for dynamics
m1 = p.m1; % mass of link 1 [kg]
m2 = p.m2; % mass of link 2 [kg]
m3 = p.m2; % mass of link 3 [kg]
m4 = p.m1; % mass of link 4 [kg]
m5 = p.m5; % mass of link 5 (trunk) [kg]

l1 = p.l1; % length of link n [m]
l2 = p.l2;
l3 = p.l2;
l4 = p.l1;
l5 = p.l5;

I1 = p.I1; % rotational inertia [kg m^2]
I2 = p.I2;
I3 = p.I2;
I4 = p.I1;
I5 = p.I5;

% torso angle should be between 70 and 90 degrees
x_head = p.l1*cos(x.th1) + p.l2*cos(x.th1+x.th2) + p.l5*cos(x.th1+x.th2+x.th5);
x_hip = p.l1*cos(x.th1) + p.l2*cos(x.th1+x.th2);
ch.add(x_head - x_hip, '>=', 0);
ch.add(x_head - x_hip, '<=', p.l5*cos(deg2rad(80)));

% dx_CoM velocity >0 (CoM term obtained from derive_CoM.m)
ch.add(-p.l1*sin(x.th1)*x.dth1 - p.l2*sin(x.th1+x.th2)*(x.dth1+x.dth2), '>=', 0);

% head should be y_head > 0.9 m
ch.add(p.l1*sin(x.th1) + p.l2*sin(x.th1+x.th2) + p.l5*sin(x.th1+x.th2+x.th5), '>=', 0.9);

% adding an virtual obstacle to avoid foot dragging
x_sw = p.l1*cos(x.th1) + p.l2*cos(x.th1+x.th2) + p.l2*cos(x.th1+x.th2+x.th3) + p.l1*cos(x.th1+x.th2+x.th3+x.th4);
y_sw = p.l1*sin(x.th1) + p.l2*sin(x.th1+x.th2) + p.l2*sin(x.th1+x.th2+x.th3) + p.l1*sin(x.th1+x.th2+x.th3+x.th4);

% ch.add( ((x_sw - p.d_obs)^2)/(p.w_obs^2) + (y_sw^2)/(p.h_obs^2), '>=', 1); % avoid the ellpse obstacle

 if k == 1
     % constraints for the initial point
     
     % swing leg starts on the ground: y_sw = 0
     ch.add(norm(p.l1*sin(x.th1) + p.l2*sin(x.th1+x.th2) + p.l2*sin(x.th1+x.th2+x.th3) + p.l1*sin(x.th1+x.th2+x.th3+x.th4))^2);
     
     % recording the initial x_hip value
     ch.add(p.l1*cos(x.th1) + p.l2*cos(x.th1+x.th2),'==', p.x_hip_I);
     
     % recording the initial states
     ch.add(norm(x.th1 - p.th1_I)^2);
     ch.add(norm(x.th2 - p.th2_I)^2);
     ch.add(norm(x.th3 - p.th3_I)^2);
     ch.add(norm(x.th4 - p.th4_I)^2);
     ch.add(norm(x.th5 - p.th5_I)^2);
     
     ch.add(norm(x.dth1 - p.dth1_I)^2);
     ch.add(norm(x.dth2 - p.dth2_I)^2);
     ch.add(norm(x.dth3 - p.dth3_I)^2);
     ch.add(norm(x.dth4 - p.dth4_I)^2);
     ch.add(norm(x.dth5 - p.dth5_I)^2);
     
     % recording x_CoM_I
     x1 = 0;
     ch.add(norm((m4*(x1 + (l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3)) + m1*(x1 + (l1*cos(th1))/2) + m3*(x1 + l2*cos(th1 + th2) + l1*cos(th1) + (l3*cos(th1 + th2 + th3))/2) + m5*(x1 + l2*cos(th1 + th2) + l1*cos(th1) + (l5*cos(th1 + th2 + th5))/2) + m2*(x1 + (l2*cos(th1 + th2))/2 + l1*cos(th1)))/(m1 + m2 + m3 + m4 + m5) - p.x_CoM_I)^2);
     
 elseif k == K
     % constraints for the final point
     
     % constraint for the step length (0.25m)
     ch.add(p.l1*cos(x.th1) + p.l2*cos(x.th1+x.th2) - p.x_hip_I, '>=', 0.25)
     ch.add(p.l1*cos(x.th1) + p.l2*cos(x.th1+x.th2) - p.x_hip_I, '<=', 0.25+1e-3)
     
%      ch.add(p.l1*cos(x.th1) + p.l2*cos(x.th1+x.th2) - p.x_hip_I, '==', 0.25)
     
     % swing leg should end on the ground
     ch.add(p.l1*sin(x.th1) + p.l2*sin(x.th1+x.th2) + p.l2*sin(x.th1+x.th2+x.th3) + p.l1*sin(x.th1+x.th2+x.th3+x.th4), '<=', 0);
     ch.add(p.l1*sin(x.th1) + p.l2*sin(x.th1+x.th2) + p.l2*sin(x.th1+x.th2+x.th3) + p.l1*sin(x.th1+x.th2+x.th3+x.th4), '>=', -1e-3);
     
     % recording x_CoM_F
     x1 = 0;
     ch.add(norm((m4*(x1 + (l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3)) + m1*(x1 + (l1*cos(th1))/2) + m3*(x1 + l2*cos(th1 + th2) + l1*cos(th1) + (l3*cos(th1 + th2 + th3))/2) + m5*(x1 + l2*cos(th1 + th2) + l1*cos(th1) + (l5*cos(th1 + th2 + th5))/2) + m2*(x1 + (l2*cos(th1 + th2))/2 + l1*cos(th1)))/(m1 + m2 + m3 + m4 + m5) - p.x_CoM_F)^2);
     ch.add(norm(x.time - p.time_F)^2);
     
     %%%%% cyclic walking constraints %%%%%
     
     % impact map
     
     dq_minus = [dth1;dth2;dth3;dth4;dth5];
     
     J_Psw_tip_minus = [- l4*sin(th1 + th2 + th3 + th4) - l2*sin(th1 + th2) - l1*sin(th1) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l2*sin(th1 + th2) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l3*sin(th1 + th2 + th3), -l4*sin(th1 + th2 + th3 + th4), 0;
         l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3),   l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3),   l4*cos(th1 + th2 + th3 + th4) + l3*cos(th1 + th2 + th3),  l4*cos(th1 + th2 + th3 + th4), 0];
     
     MM_minus = [
         I1 + I2 + I3 + I4 + I5 + (l1^2*m1)/4 + l1^2*m2 + l1^2*m3 + (l2^2*m2)/4 + l1^2*m4 + l2^2*m3 + l1^2*m5 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l1*l3*m3*cos(th2 + th3) + 2*l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + l1*l5*m5*cos(th2 + th5) + l1*l2*m2*cos(th2) + 2*l1*l2*m3*cos(th2) + 2*l1*l2*m4*cos(th2) + 2*l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + l1*l4*m4*cos(th2 + th3 + th4), I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2, I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2, I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2, I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2;
         I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                               I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l2*l4*m4*cos(th3 + th4) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5),                                                                                             I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4),                                     I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2,                               (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5;
         I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                                                                                                                 I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4),                                                                                                                                                                       I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + l3*l4*m4*cos(th4),                                                                   (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4,                                                                      0;
         I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                                                                                                                                                                                      I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2,                                                                                                                                                                                                (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4,                                                                                           (m4*l4^2)/4 + I4,                                                                      0;
         I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2,                                                                                                                                                                                                                                                                                                                                                                                                    (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5,                                                                                                                                                                                                                                       0,                                                                                                          0,                                                       (m5*l5^2)/4 + I5];
     
     dq_after = (eye(5)-inv(MM_minus)*J_Psw_tip_minus.'*inv(J_Psw_tip_minus*inv(MM_minus)*J_Psw_tip_minus.')*J_Psw_tip_minus)*dq_minus;
     dth1_after = dq_after(1);
     dth2_after = dq_after(2);
     dth3_after = dq_after(3);
     dth4_after = dq_after(4);
     dth5_after = dq_after(5);
          
     % initial angles should be the same for cyclic walking
     ch.add(norm( (x.th1 + x.th2 + x.th3 + x.th4 - pi) - p.th1_I)^2);
     ch.add(norm( (-x.th4 + 2*pi) - p.th2_I)^2);
     ch.add(norm( (-x.th3 - 2*pi) - p.th3_I)^2);
     ch.add(norm( (-x.th2 + 2*pi) - p.th4_I)^2);
     ch.add(norm( (x.th5 - x.th3 + pi - 2*pi) - p.th5_I)^2);
     
     % initial and final velocities should be the same
     ch.add(norm( (dth1_after + dth2_after + dth3_after + dth4_after) - p.dth1_I)^2);
     ch.add(norm( (-dth4_after) - p.dth2_I)^2);
     ch.add(norm( (-dth3_after) - p.dth3_I)^2);
     ch.add(norm( (-dth2_after) - p.dth4_I)^2);
     ch.add(norm( (dth5_after - dth3_after) - p.dth5_I)^2);
     
     %%%%%----------------------------%%%%%
 else
     % constraints for in-between points
     
     % swing leg should be above ground: y_sw > 0
     ch.add(p.l1*sin(x.th1) + p.l2*sin(x.th1+x.th2) + p.l2*sin(x.th1+x.th2+x.th3) + p.l1*sin(x.th1+x.th2+x.th3+x.th4), '>=', 0);
     
     % swing leg horizontal velocity should be positive: dx_sw > 0
     ch.add(-(p.l1*sin(x.th1)*x.dth1 + p.l2*sin(x.th1+x.th2)*(x.dth1+x.dth2) + p.l2*sin(x.th1+x.th2+x.th3)*(x.dth1+x.dth2+x.dth3) + p.l1*sin(x.th1+x.th2+x.th3+x.th4)*(x.dth1+x.dth2+x.dth3+x.dth4)), '>=', 0);
 end
 
end

function ss_pathcosts(ch, x, z, u, p)

tau = [u.u2; u.u3; u.u4; u.u5];
% ch.add((0.5*(tau.'*tau))*p.time_F)

% Specific Resistance

% % States
% th1 = x.th1;
% th2 = x.th2;
% th3 = x.th3;
% th4 = x.th4;
% th5 = x.th5;
% 
% dth1 = x.dth1;
% dth2 = x.dth2;
% dth3 = x.dth3;
% dth4 = x.dth4;
% dth5 = x.dth5;
% 
% %%%%% Parameters for dynamics %%%%%
% m1 = p.m1; % mass of link 1 [kg]
% m2 = p.m2; % mass of link 2 [kg]
% m3 = p.m2; % mass of link 3 [kg]
% m4 = p.m1; % mass of link 4 [kg]
% m5 = p.m5; % mass of link 5 (trunk) [kg]
% 
% l1 = p.l1; % length of link n [m]
% l2 = p.l2;
% l3 = p.l2;
% l4 = p.l1;
% l5 = p.l5;
% 
% I1 = p.I1; % rotational inertia [kg m^2]
% I2 = p.I2;
% I3 = p.I2;
% I4 = p.I1;
% I5 = p.I5;
% 
% gravi = p.g; % gravitational acceleration [m/s^2]
% 
% % dxG = - (dth3*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l3*sin(th1 + th2 + th3)) + (l3*m3*sin(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) - (dth1*(m3*(l2*sin(th1 + th2) + l1*sin(th1) + (l3*sin(th1 + th2 + th3))/2) + m5*(l2*sin(th1 + th2) + l1*sin(th1) + (l5*sin(th1 + th2 + th5))/2) + m2*((l2*sin(th1 + th2))/2 + l1*sin(th1)) + m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l2*sin(th1 + th2) + l1*sin(th1) + l3*sin(th1 + th2 + th3)) + (l1*m1*sin(th1))/2))/(m1 + m2 + m3 + m4 + m5) - (dth2*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)) + m3*(l2*sin(th1 + th2) + (l3*sin(th1 + th2 + th3))/2) + m5*(l2*sin(th1 + th2) + (l5*sin(th1 + th2 + th5))/2) + (l2*m2*sin(th1 + th2))/2))/(m1 + m2 + m3 + m4 + m5) - (dth4*l4*m4*sin(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) - (dth5*l5*m5*sin(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5));
% M_total = m1 + m2 + m3 + m4 + m5;
% power = abs(x.u2*dth2) + abs(x.u3*dth3) + abs(x.u4*dth4) + abs(x.u5*dth5);
% avg_power = power/p.time_F;
% 
% % new Specific Resistance using parameters.
% avg_vel = (p.x_CoM_F - p.x_CoM_I)/p.time_F;
% ch.add(100*avg_power/(M_total*gravi*avg_vel));

end


%% Double stance phase
function doubleStance_vars(sh)
% State = [th1; th2; th3; th4; th5; dth1; dth2; dth3; dth4; dth5]
sh.addState('th1', 'lb', 0, 'ub', pi); % add a state with lower and upper bounds
sh.addState('th2', 'lb', deg2rad(5), 'ub', deg2rad(22.5));
sh.addState('th3', 'lb', -2*pi, 'ub', 0);
sh.addState('th4', 'lb', deg2rad(270), 'ub', deg2rad(360));
sh.addState('th5', 'lb', -pi, 'ub', pi);

sh.addState('dth1', 'lb', -10, 'ub', 10);
sh.addState('dth2', 'lb', -10, 'ub', 10);
sh.addState('dth3', 'lb', -10, 'ub', 10);
sh.addState('dth4', 'lb', -10, 'ub', 10);
sh.addState('dth5', 'lb', -10, 'ub', 10);

% adding the control as a state so that GRF can be calculated in gridconstrains
% sh.addState('u1');  % joint 1 torque
sh.addControl('u2');  % 
sh.addControl('u3');  % 
sh.addControl('u4');  % 
sh.addControl('u5');  % 

sh.addState('time', 'lb', 0, 'ub', 2);

% % adding the time derivative of the control as the control
% % sh.addControl('du1');  % time derivative of joint 1 torque
% sh.addControl('du2');  % 
% sh.addControl('du3');  % 
% sh.addControl('du4');  % 
% sh.addControl('du5');  % 

sh.addParameter('m1');
sh.addParameter('m2');
sh.addParameter('m5');

sh.addParameter('l1');
sh.addParameter('l2');
sh.addParameter('l5');

sh.addParameter('I1');
sh.addParameter('I2');
sh.addParameter('I5');

sh.addParameter('g');

sh.addParameter('d_obs');
% sh.addParameter('r_obs');
sh.addParameter('h_obs');
sh.addParameter('w_obs');

% Collocation Parameters
sh.addParameter('x_hip_I');

sh.addParameter('th1_I');
sh.addParameter('th2_I');
sh.addParameter('th3_I');
sh.addParameter('th4_I');
sh.addParameter('th5_I');

sh.addParameter('dth1_I');
sh.addParameter('dth2_I');
sh.addParameter('dth3_I');
sh.addParameter('dth4_I');
sh.addParameter('dth5_I');

sh.addParameter('x_CoM_I');
sh.addParameter('x_CoM_F');
sh.addParameter('time_F');
end

function doubleStance_ode(sh,x,~,u,p)

% States
th1 = x.th1;
th2 = x.th2;
th3 = x.th3;
th4 = x.th4;
th5 = x.th5;

dth1 = x.dth1;
dth2 = x.dth2;
dth3 = x.dth3;
dth4 = x.dth4;
dth5 = x.dth5;

%%%%% Parameters for dynamics %%%%%
m1 = p.m1; % mass of link 1 [kg]
m2 = p.m2; % mass of link 2 [kg]
m3 = p.m2; % mass of link 3 [kg]
m4 = p.m1; % mass of link 4 [kg]
m5 = p.m5; % mass of link 5 (trunk) [kg]

l1 = p.l1; % length of link n [m]
l2 = p.l2;
l3 = p.l2;
l4 = p.l1;
l5 = p.l5;

I1 = p.I1; % rotational inertia [kg m^2]
I2 = p.I2;
I3 = p.I2;
I4 = p.I1;
I5 = p.I5;

gravi = p.g; % gravitational acceleration [m/s^2]

% Inertia Matrix
MM = [
    I1 + I2 + I3 + I4 + I5 + (l1^2*m1)/4 + l1^2*m2 + l1^2*m3 + (l2^2*m2)/4 + l1^2*m4 + l2^2*m3 + l1^2*m5 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l1*l3*m3*cos(th2 + th3) + 2*l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + l1*l5*m5*cos(th2 + th5) + l1*l2*m2*cos(th2) + 2*l1*l2*m3*cos(th2) + 2*l1*l2*m4*cos(th2) + 2*l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + l1*l4*m4*cos(th2 + th3 + th4), I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2, I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2, I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2, I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2;
    I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                               I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l2*l4*m4*cos(th3 + th4) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5),                                                                                             I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4),                                     I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2,                               (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5;
    I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                                                                                                                 I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4),                                                                                                                                                                       I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + l3*l4*m4*cos(th4),                                                                   (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4,                                                                      0;
    I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                                                                                                                                                                                      I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2,                                                                                                                                                                                                (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4,                                                                                           (m4*l4^2)/4 + I4,                                                                      0;
    I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2,                                                                                                                                                                                                                                                                                                                                                                                                    (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5,                                                                                                                                                                                                                                       0,                                                                                                          0,                                                       (m5*l5^2)/4 + I5];


% Centrifugal force and Coriolis Force Vector
CC =  [
 - (dth2^2*l1*l3*m3*sin(th2 + th3))/2 - dth2^2*l1*l3*m4*sin(th2 + th3) - (dth3^2*l1*l3*m3*sin(th2 + th3))/2 - dth3^2*l1*l3*m4*sin(th2 + th3) - (dth2^2*l1*l5*m5*sin(th2 + th5))/2 - (dth3^2*l2*l4*m4*sin(th3 + th4))/2 - (dth4^2*l2*l4*m4*sin(th3 + th4))/2 - (dth5^2*l1*l5*m5*sin(th2 + th5))/2 - (dth2^2*l1*l2*m2*sin(th2))/2 - dth2^2*l1*l2*m3*sin(th2) - dth2^2*l1*l2*m4*sin(th2) - dth2^2*l1*l2*m5*sin(th2) - (dth3^2*l2*l3*m3*sin(th3))/2 - dth3^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 - (dth5^2*l2*l5*m5*sin(th5))/2 - (dth2^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - (dth3^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - (dth4^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - dth1*dth2*l1*l3*m3*sin(th2 + th3) - 2*dth1*dth2*l1*l3*m4*sin(th2 + th3) - dth1*dth3*l1*l3*m3*sin(th2 + th3) - 2*dth1*dth3*l1*l3*m4*sin(th2 + th3) - dth2*dth3*l1*l3*m3*sin(th2 + th3) - 2*dth2*dth3*l1*l3*m4*sin(th2 + th3) - dth1*dth2*l1*l5*m5*sin(th2 + th5) - dth1*dth3*l2*l4*m4*sin(th3 + th4) - dth1*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth3*l2*l4*m4*sin(th3 + th4) - dth2*dth4*l2*l4*m4*sin(th3 + th4) - dth1*dth5*l1*l5*m5*sin(th2 + th5) - dth3*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth5*l1*l5*m5*sin(th2 + th5) - dth1*dth2*l1*l2*m2*sin(th2) - 2*dth1*dth2*l1*l2*m3*sin(th2) - 2*dth1*dth2*l1*l2*m4*sin(th2) - 2*dth1*dth2*l1*l2*m5*sin(th2) - dth1*dth3*l2*l3*m3*sin(th3) - 2*dth1*dth3*l2*l3*m4*sin(th3) - dth2*dth3*l2*l3*m3*sin(th3) - 2*dth2*dth3*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4) - dth1*dth5*l2*l5*m5*sin(th5) - dth2*dth5*l2*l5*m5*sin(th5) - dth1*dth2*l1*l4*m4*sin(th2 + th3 + th4) - dth1*dth3*l1*l4*m4*sin(th2 + th3 + th4) - dth1*dth4*l1*l4*m4*sin(th2 + th3 + th4) - dth2*dth3*l1*l4*m4*sin(th2 + th3 + th4) - dth2*dth4*l1*l4*m4*sin(th2 + th3 + th4) - dth3*dth4*l1*l4*m4*sin(th2 + th3 + th4);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        (dth1^2*l1*l3*m3*sin(th2 + th3))/2 + dth1^2*l1*l3*m4*sin(th2 + th3) + (dth1^2*l1*l5*m5*sin(th2 + th5))/2 - (dth3^2*l2*l4*m4*sin(th3 + th4))/2 - (dth4^2*l2*l4*m4*sin(th3 + th4))/2 + (dth1^2*l1*l2*m2*sin(th2))/2 + dth1^2*l1*l2*m3*sin(th2) + dth1^2*l1*l2*m4*sin(th2) + dth1^2*l1*l2*m5*sin(th2) - (dth3^2*l2*l3*m3*sin(th3))/2 - dth3^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 - (dth5^2*l2*l5*m5*sin(th5))/2 + (dth1^2*l1*l4*m4*sin(th2 + th3 + th4))/2 - dth1*dth3*l2*l4*m4*sin(th3 + th4) - dth1*dth4*l2*l4*m4*sin(th3 + th4) - dth2*dth3*l2*l4*m4*sin(th3 + th4) - dth2*dth4*l2*l4*m4*sin(th3 + th4) - dth3*dth4*l2*l4*m4*sin(th3 + th4) - dth1*dth3*l2*l3*m3*sin(th3) - 2*dth1*dth3*l2*l3*m4*sin(th3) - dth2*dth3*l2*l3*m3*sin(th3) - 2*dth2*dth3*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4) - dth1*dth5*l2*l5*m5*sin(th5) - dth2*dth5*l2*l5*m5*sin(th5);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            (dth1^2*l1*l3*m3*sin(th2 + th3))/2 + dth1^2*l1*l3*m4*sin(th2 + th3) + (dth1^2*l2*l4*m4*sin(th3 + th4))/2 + (dth2^2*l2*l4*m4*sin(th3 + th4))/2 + (dth1^2*l2*l3*m3*sin(th3))/2 + dth1^2*l2*l3*m4*sin(th3) + (dth2^2*l2*l3*m3*sin(th3))/2 + dth2^2*l2*l3*m4*sin(th3) - (dth4^2*l3*l4*m4*sin(th4))/2 + (dth1^2*l1*l4*m4*sin(th2 + th3 + th4))/2 + dth1*dth2*l2*l4*m4*sin(th3 + th4) + dth1*dth2*l2*l3*m3*sin(th3) + 2*dth1*dth2*l2*l3*m4*sin(th3) - dth1*dth4*l3*l4*m4*sin(th4) - dth2*dth4*l3*l4*m4*sin(th4) - dth3*dth4*l3*l4*m4*sin(th4);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      (l4*m4*(dth1^2*l2*sin(th3 + th4) + dth2^2*l2*sin(th3 + th4) + dth1^2*l3*sin(th4) + dth2^2*l3*sin(th4) + dth3^2*l3*sin(th4) + dth1^2*l1*sin(th2 + th3 + th4) + 2*dth1*dth2*l2*sin(th3 + th4) + 2*dth1*dth2*l3*sin(th4) + 2*dth1*dth3*l3*sin(th4) + 2*dth2*dth3*l3*sin(th4)))/2;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           (l5*m5*(dth1^2*l1*sin(th2 + th5) + dth1^2*l2*sin(th5) + dth2^2*l2*sin(th5) + 2*dth1*dth2*l2*sin(th5)))/2];

% Gravity Vector
GG = [
    
 
 (m3*(l2*cos(th1 + th2) + l1*cos(th1) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + l1*cos(th1) + (l5*cos(th1 + th2 + th5))/2) + m2*((l2*cos(th1 + th2))/2 + l1*cos(th1)) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3)) + (l1*m1*cos(th1))/2)*gravi;
                                                                                (m3*(l2*cos(th1 + th2) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + (l5*cos(th1 + th2 + th5))/2) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) + (l2*m2*cos(th1 + th2))/2)*gravi;
                                                                                                                                                                                                            (m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l3*cos(th1 + th2 + th3)) + (l3*m3*cos(th1 + th2 + th3))/2)*gravi;
                                                                                                                                                                                                                                                                         ((l4*m4*cos(th1 + th2 + th3 + th4))/2)*gravi;
                                                                                                                                                                                                                                                                               ((l5*m5*cos(th1 + th2 + th5))/2)*gravi];
% calculating the accelerations
% tau = [x.u1; x.u2; x.u3; x.u4; x.u5 - x.u3];
tau = [0; u.u2; u.u3; u.u4; u.u5 - u.u3];
ddq = inv(MM)*(tau - CC - GG);

sh.setODE('th1', x.dth1);
sh.setODE('th2', x.dth2);
sh.setODE('th3', x.dth3);
sh.setODE('th4', x.dth4);
sh.setODE('th5', x.dth5);

sh.setODE('dth1', ddq(1) );
sh.setODE('dth2', ddq(2) );
sh.setODE('dth3', ddq(3) );
sh.setODE('dth4', ddq(4) );
sh.setODE('dth5', ddq(5) );

% sh.setODE('u2', u.du2 );
% sh.setODE('u3', u.du3 );
% sh.setODE('u4', u.du4 );
% sh.setODE('u5', u.du5 );

sh.setODE('time', 1); % why this is set to 1, not sure. probably this increases t as much as the increment
end

function ds_gridconstraints(ch, k, K, x, p)

if k == 1

elseif k == K
 
end

end

function ds_pathcosts(ch, x, z, u, p)

tau = [u.u2; u.u3; u.u4; u.u5];
% ch.add((0.5*(tau.'*tau))*p.time_F)

end

function stage_transition(ch, x0, xF, param0, paramF)
% x0 current stage
% xF previous stage

% States
th1 = xF.th1;
th2 = xF.th2;
th3 = xF.th3;
th4 = xF.th4;
th5 = xF.th5;

dth1 = xF.dth1;
dth2 = xF.dth2;
dth3 = xF.dth3;
dth4 = xF.dth4;
dth5 = xF.dth5;

% Parameters for dynamics
m1 = paramF.m1; % mass of link 1 [kg]
m2 = paramF.m2; % mass of link 2 [kg]
m3 = paramF.m2; % mass of link 3 [kg]
m4 = paramF.m1; % mass of link 4 [kg]
m5 = paramF.m5; % mass of link 5 (trunk) [kg]

l1 = paramF.l1; % length of link n [m]
l2 = paramF.l2;
l3 = paramF.l2;
l4 = paramF.l1;
l5 = paramF.l5;

I1 = paramF.I1; % rotational inertia [kg m^2]
I2 = paramF.I2;
I3 = paramF.I2;
I4 = paramF.I1;
I5 = paramF.I5;

% impact map
     
 dq_minus = [dth1;dth2;dth3;dth4;dth5];
 
 J_Psw_tip_minus = [- l4*sin(th1 + th2 + th3 + th4) - l2*sin(th1 + th2) - l1*sin(th1) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l2*sin(th1 + th2) - l3*sin(th1 + th2 + th3), - l4*sin(th1 + th2 + th3 + th4) - l3*sin(th1 + th2 + th3), -l4*sin(th1 + th2 + th3 + th4), 0;
     l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3),   l4*cos(th1 + th2 + th3 + th4) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3),   l4*cos(th1 + th2 + th3 + th4) + l3*cos(th1 + th2 + th3),  l4*cos(th1 + th2 + th3 + th4), 0];
 
 MM_minus = [
     I1 + I2 + I3 + I4 + I5 + (l1^2*m1)/4 + l1^2*m2 + l1^2*m3 + (l2^2*m2)/4 + l1^2*m4 + l2^2*m3 + l1^2*m5 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l1*l3*m3*cos(th2 + th3) + 2*l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + l1*l5*m5*cos(th2 + th5) + l1*l2*m2*cos(th2) + 2*l1*l2*m3*cos(th2) + 2*l1*l2*m4*cos(th2) + 2*l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + l1*l4*m4*cos(th2 + th3 + th4), I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2, I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2, I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2, I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2;
     I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + l2*l4*m4*cos(th3 + th4) + (l1*l5*m5*cos(th2 + th5))/2 + (l1*l2*m2*cos(th2))/2 + l1*l2*m3*cos(th2) + l1*l2*m4*cos(th2) + l1*l2*m5*cos(th2) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5) + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                               I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + (l3^2*m3)/4 + l2^2*m5 + l3^2*m4 + (l4^2*m4)/4 + (l5^2*m5)/4 + l2*l4*m4*cos(th3 + th4) + l2*l3*m3*cos(th3) + 2*l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + l2*l5*m5*cos(th5),                                                                                             I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4),                                     I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2,                               (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5;
     I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l1*l3*m3*cos(th2 + th3))/2 + l1*l3*m4*cos(th2 + th3) + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4) + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                                                                                                                 I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l2*l3*m3*cos(th3))/2 + l2*l3*m4*cos(th3) + l3*l4*m4*cos(th4),                                                                                                                                                                       I3 + I4 + (l3^2*m3)/4 + l3^2*m4 + (l4^2*m4)/4 + l3*l4*m4*cos(th4),                                                                   (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4,                                                                      0;
     I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2 + (l1*l4*m4*cos(th2 + th3 + th4))/2,                                                                                                                                                                                                                                                                                                                                                                      I4 + (l4^2*m4)/4 + (l2*l4*m4*cos(th3 + th4))/2 + (l3*l4*m4*cos(th4))/2,                                                                                                                                                                                                (m4*l4^2)/4 + (l3*m4*cos(th4)*l4)/2 + I4,                                                                                           (m4*l4^2)/4 + I4,                                                                      0;
     I5 + (l5^2*m5)/4 + (l1*l5*m5*cos(th2 + th5))/2 + (l2*l5*m5*cos(th5))/2,                                                                                                                                                                                                                                                                                                                                                                                                    (m5*l5^2)/4 + (l2*m5*cos(th5)*l5)/2 + I5,                                                                                                                                                                                                                                       0,                                                                                                          0,                                                       (m5*l5^2)/4 + I5];
 
 dq_after = (eye(5)-inv(MM_minus)*J_Psw_tip_minus.'*inv(J_Psw_tip_minus*inv(MM_minus)*J_Psw_tip_minus.')*J_Psw_tip_minus)*dq_minus;
 dth1_after = dq_after(1);
 dth2_after = dq_after(2);
 dth3_after = dq_after(3);
 dth4_after = dq_after(4);
 dth5_after = dq_after(5);

 % Stage Transition Constraints
ch.add(x0.time, '==', xF.time);

ch.add(x0.th1, '==', xF.th1 + xF.th2 + xF.th3 + xF.th4 - pi);
ch.add(x0.th2, '==', -xF.th4 + 2*pi);
ch.add(x0.th3, '==', -xF.th3 - 2*pi);
ch.add(x0.th4, '==', -xF.th2 + 2*pi);
ch.add(x0.th5, '==', xF.th5 - xF.th3 + pi - 2*pi);

ch.add(x0.dth1, '==', dth1_after);
ch.add(x0.dth2, '==', dth2_after);
ch.add(x0.dth3, '==', dth3_after);
ch.add(x0.dth4, '==', dth4_after);
ch.add(x0.dth5, '==', dth5_after);

end
