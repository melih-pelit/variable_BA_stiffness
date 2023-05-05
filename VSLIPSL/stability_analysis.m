function analysis(simout, param, x1_out, step_no, rt_VLO, time)
%% Stability Analysis

%% Parameters

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

%%
%--- Limit Cycle-----------------------------------------------------------
nt = length(simout);
search_step = 1;
prev_step_no = -1;

figure()

k = 1;
start_frame = 1;
for i = start_frame:search_step:nt
    
    CoM = calculate_com(simout(i, :), param, x1_out(i), [0;0;0;0;0]);
    
    z_CoM(k) = CoM(2);
    dz_CoM(k) = CoM(4);
          
    k = k + 1;
end

plot(z_CoM, dz_CoM)
hold on

% writing the step number when if changes
k = 1;
for i = start_frame:search_step:nt
    
    if prev_step_no < step_no(k)
%         txt = num2str(step_no(k));
%         text(z_CoM(k), dz_CoM(k), txt)
        prev_step_no = step_no(k);
        plot(z_CoM(k), dz_CoM(k), 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'r');
    end
    k = k + 1;
end

title('Limit Cycle');
xlabel('CoM z');
ylabel('CoM dz');
%--------------------------------------------------------------------------

%% Total CoM Energy Plot @VLO

% E_CoM(:,1) = rt_VLO(1,4,:);

% figure()
% plot(poincare_step_no, E_CoM)
% title('Total CoM Energy Plot @VLO');
% xlabel('Step No');
% ylabel('CoM Energy');

%% Poincare Map with rt_VLO (Stability Analysis)
% Detecting data in Simulink instead of matlab main file

theta_P = [];
z_P = [];
theta_P(:,1) = rt_VLO(1,1,:);
z_P(:,1) = rt_VLO(1,2,:);
poincare_step_no(:,1) = rt_VLO(1,3,:);
E_CoM(:,1) = rt_VLO(1,4,:);

% plotting Poincare Map
figure()
length_poincare = length(theta_P);

plot3(theta_P, z_P, E_CoM,'--')
hold on
for i = 1:1:(length_poincare - 1)
    %     plot3([theta_P(i), theta_P(i+1)], [z_P(i), z_P(i+1)], '--.k')
    plot3(theta_P(i), z_P(i), E_CoM(i),'.b')
%     txt = num2str(poincare_step_no(i));
%     text(theta_P(i), z_P(i), txt)  
end

title('Poincare Map');
xlabel('\theta_P');
ylabel('z_P');
zlabel('Energy of CoM');
grid on
view(0,90)


%% Distance between VLO points on Poincare map

% two dimensional Poincare Map (theta_P, z_P)
k = 1;
for i = 1:1:(length_poincare - 1)
    vector_pc = [(theta_P(i+1) - theta_P(i))/mean(theta_P), (z_P(i+1) - z_P(i))/mean(z_P), (E_CoM(i+1) - E_CoM(i))/mean(E_CoM)]; % vector between two adjacent points in Poincare Map
    d_pc(k) = norm(vector_pc); % distance between adjacent Poincare Points
    k = k + 1;
end
figure()
plot(poincare_step_no(1:(length(poincare_step_no) - 1)), d_pc)
title('Distance between VLO points on Poincare map');
xlabel('Step No');
ylabel('Distance');

%% Total Energy of CoM Analysis
nt = length(simout);
search_step = 1;
k = 1;
for i = 1:search_step:nt
    CoM_energy.time(k) = time(i);
    CoM_energy.energy(k) = calculate_CoM_energy(simout(i, :), param);
    k = k + 1;
end

% plotting
figure()
plot(CoM_energy.time, CoM_energy.energy);
title('CoM Energy')
end
