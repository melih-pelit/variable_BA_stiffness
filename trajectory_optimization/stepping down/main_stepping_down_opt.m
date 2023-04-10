% 2022.03.09 5LinkWalk_OpenOCL.m
% Generating optimum inputs and walking trajectories for 5 Link 
% Bipedal Model using OpenOCL (4 input torques)
% Mustafa Melih PELIT

% clc
clear all

ocp = ocl.Problem([], 'vars', @singleStance_vars, 'dae', @singleStance_ode, 'gridconstraints', @ss_gridconstraints, 'pathcosts', @ss_pathcosts, 'N', 8, 'd', 3);

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

% setting OpenOCL parameters
ocp.setParameter('m1'   , param.m1);
ocp.setParameter('m2'   , param.m2);
ocp.setParameter('m5'   , param.m5);

ocp.setParameter('l1'   , param.l1);
ocp.setParameter('l2'   , param.l2);
ocp.setParameter('l5'   , param.l5);

ocp.setParameter('I1'   , param.I1);
ocp.setParameter('I2'   , param.I2);
ocp.setParameter('I5'   , param.I5);

ocp.setParameter('g'   , param.g);

%%
walking_filename = 'OpenOCLtraj\ocl_5link_traj2023-04-08-15-59.mat';
load(walking_filename);

% setting the initial state
ocp.setInitialBounds( 'time',   0.0);

ocp.setInitialBounds( 'th1', ocl_traj.ss.simout(end,1))
ocp.setInitialBounds( 'th2', ocl_traj.ss.simout(end,2))
ocp.setInitialBounds( 'th3', ocl_traj.ss.simout(end,3))
ocp.setInitialBounds( 'th4', ocl_traj.ss.simout(end,4))
ocp.setInitialBounds( 'th5', ocl_traj.ss.simout(end,5))

ocp.setInitialBounds( 'dth1', ocl_traj.ss.simout(end,6))
ocp.setInitialBounds( 'dth2', ocl_traj.ss.simout(end,7))
ocp.setInitialBounds( 'dth3', ocl_traj.ss.simout(end,8))
ocp.setInitialBounds( 'dth4', ocl_traj.ss.simout(end,9))
ocp.setInitialBounds( 'dth5', ocl_traj.ss.simout(end,10))

%%%%% Collocation Parameters %%%%%

% parameters for recording initial and final states
ocp.setBounds('th1_I', -2*pi, 2*pi);
ocp.setBounds('th2_I', -2*pi, 2*pi);
ocp.setBounds('th3_I', -2*pi, 2*pi);
ocp.setBounds('th4_I', -2*pi, 2*pi);
ocp.setBounds('th5_I', -2*pi, 2*pi);


%% Solve

sol = ocp.getInitialGuess();

% %%% using previous solutions
% ocp.getInitialGuess(); % this line is necessary to use old trajectories as initial guess
load('results\landing_ocl_result2023-04-10-10-10.mat')

[sol,times] = ocp.solve(sol);
sr(1) = calc_sr(sol);

j = 2;
%%
rpt = 1; % how many times to repeat?
for i=j:j+(rpt-1)
    [sol,times] = ocp.solve(sol);
    sr(j) = calc_sr(sol)
    
    simout = [sol.states.th1.value; sol.states.th2.value; sol.states.th3.value; sol.states.th4.value; sol.states.th5.value; sol.states.dth1.value; sol.states.dth2.value; sol.states.dth3.value; sol.states.dth4.value; sol.states.dth5.value]';
    u = [sol.controls.u2.value; sol.controls.u3.value; sol.controls.u4.value; sol.controls.u5.value]';
    time = sol.states.time.value;
    
    j = j+1;
end
%% Recording the solution
f_record = 1;
if f_record == 1
    filename = sprintf('landing_ocl_result%s.mat', datestr(now,'yyyy-mm-dd-HH-MM'));
    subfolder = 'results';
    save(fullfile(subfolder,filename),'sol')
end
%% Animation 

f_video = 1;
f_pause = 1;

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
plot(time, dy_sw)

% center of mass


%% Calculating specific resistance
specific_resistance = calc_sr(sol);

%%

function singleStance_vars(sh)
% State = [th1; th2; th3; th4; th5; dth1; dth2; dth3; dth4; dth5]
sh.addState('th1', 'lb', deg2rad(45), 'ub', deg2rad(135)); % add a state with lower and upper bounds
sh.addState('th2', 'lb', deg2rad(5), 'ub', deg2rad(90));
sh.addState('th3', 'lb', deg2rad(135), 'ub', deg2rad(225));
sh.addState('th4', 'lb', deg2rad(270), 'ub', deg2rad(355));
sh.addState('th5', 'lb', -deg2rad(45), 'ub', deg2rad(45));

% sh.addState('dth1', 'lb', -10, 'ub', 10);
% % sh.addState('dth1', 'lb', -10, 'ub', -0.5);
% sh.addState('dth2', 'lb', -10, 'ub', 10);
% sh.addState('dth3', 'lb', -10, 'ub', 10);
% sh.addState('dth4', 'lb', -10, 'ub', 20);
% sh.addState('dth5', 'lb', -10, 'ub', 10);

sh.addState('dth1');
sh.addState('dth2');
sh.addState('dth3');
sh.addState('dth4');
sh.addState('dth5');

% adding the control as a state so that GRF can be calculated in gridconstrains
% sh.addState('u1');  % joint 1 torque
sh.addControl('u2');  % 
sh.addControl('u3');  % 
sh.addControl('u4');  % 
sh.addControl('u5');  % 

sh.addState('time', 'lb', 0, 'ub', 2);

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

% Collocation Parameters

sh.addParameter('th1_I');
sh.addParameter('th2_I');
sh.addParameter('th3_I');
sh.addParameter('th4_I');
sh.addParameter('th5_I');

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

ch.add(dth1, '<=', -0.5)
% ch.add(th1 + th2 + th5, '>=', deg2rad(75))
% ch.add(th1 + th2 + th5, '<=', deg2rad(105))

% dx_hip velocity >0 (CoM term obtained from derive_CoM.m)
% ch.add(-p.l1*sin(x.th1)*x.dth1 - p.l2*sin(x.th1+x.th2)*(x.dth1+x.dth2), '>=', 0);

% y velocity of the swing foot should be below a certain threshold
% ch.add( p.l1*cos(x.th1)*x.dth1 + p.l2*cos(x.th1+x.th2)*(x.dth1+x.dth2) ...
%     + p.l2*cos(x.th1+x.th2+x.th3)*(x.dth1+x.dth2+x.dth3) ...
%     + p.l1*cos(x.th1+x.th2+x.th3+x.th4)*(x.dth1+x.dth2+x.dth3+x.dth4), '<=' , -0.2);

if k == 1
     % constraints for the initial point
     
     % recording the initial x_hip value
%      ch.add(p.l1*cos(x.th1) + p.l2*cos(x.th1+x.th2),'==', p.x_hip_I);
     
     % recording the initial states
     ch.add(norm(x.th1 - p.th1_I)^2);
     ch.add(norm(x.th2 - p.th2_I)^2);
     ch.add(norm(x.th3 - p.th3_I)^2);
     ch.add(norm(x.th4 - p.th4_I)^2);
     ch.add(norm(x.th5 - p.th5_I)^2);
 
 elseif k == K
     
     % swing leg should end on the ground
     ch.add(l1*sin(th1) + l2*sin(th1+th2) + l2*sin(th1+th2+th3) + l1*sin(th1+th2+th3+th4), '<=', -0.05);
     % ch.add(l1*sin(th1) + l2*sin(th1+th2) + l2*sin(th1+th2+th3) + l1*sin(th1+th2+th3+th4), '>=', -0.051);     

     % dx_sw < 0 at touchdown
     % ch.add( -p.l1*sin(x.th1)*x.dth1 - p.l2*sin(x.th1+x.th2)*(x.dth1+x.dth2) ...
     %     - p.l2*sin(x.th1+x.th2+x.th3)*(x.dth1+x.dth2+x.dth3) ...
     %     - p.l1*sin(x.th1+x.th2+x.th3+x.th4)*(x.dth1+x.dth2+x.dth3+x.dth4), '<=' , 0);

 else
     % constraints for in-between points
    
 end
 
end

function ss_pathcosts(ch, x, z, u, p)

th1_diff = (x.th1 - p.th1_I)^2;
th2_diff = (x.th2 - p.th2_I)^2;
th3_diff = (x.th3 - p.th3_I)^2;
th4_diff = (x.th4 - p.th4_I)^2;
th5_diff = (x.th5 - p.th5_I)^2;

% ch.add(th1_diff^2 + th2_diff^2 + th3_diff^2 + th4_diff^2 + th5_diff^2);

tau = [u.u2; u.u3; u.u4; u.u5];
ch.add((th1_diff^2 + th2_diff^2 + th3_diff^2 + th4_diff^2 + th5_diff^2)*(tau.'*tau));
% ch.add((10*(tau.'*tau)))

end
