function figure_poincare(simout, param, x1_out, step_no, rt_VLO, time)

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

plot3(theta_P(2:end), z_P(2:end), E_CoM(2:end),'--')
hold on
for i = 2:1:(length_poincare - 1)
    %     plot3([theta_P(i), theta_P(i+1)], [z_P(i), z_P(i+1)], '--.k')
    plot3(theta_P(i), z_P(i), E_CoM(i),'.b')
%     txt = num2str(poincare_step_no(i));
%     text(theta_P(i)+0.001, z_P(i) - 0.0001, txt)  
end

for i = 2:1:6
    %     plot3([theta_P(i), theta_P(i+1)], [z_P(i), z_P(i+1)], '--.k')
%     plot3(theta_P(i), z_P(i), E_CoM(i),'.b')
    txt = num2str(poincare_step_no(i));
    text(theta_P(i)+0.001, z_P(i) - 0.00002, txt)  
end

title('Poincare Map');
xlabel('\theta_{P, VLO}');
ylabel('y_{VLO}');
zlabel('Energy of CoM');
grid on
view(0,90)
% xlim([0.14,0.27])


% set(gcf, 'color', 'none');
% set(gca, 'color', 'none');
% export_fig poincare.png -m3

% zoomed in poincare map

figure
plot3(theta_P(2:end), z_P(2:end), E_CoM(2:end),'--')

hold on
for i = 2:1:(length_poincare - 1)
    %     plot3([theta_P(i), theta_P(i+1)], [z_P(i), z_P(i+1)], '--.k')
    plot3(theta_P(i), z_P(i), E_CoM(i),'.b')
%     txt = num2str(poincare_step_no(i));
%     text(theta_P(i), z_P(i), txt)  
end

for i = 2:1:12
    %     plot3([theta_P(i), theta_P(i+1)], [z_P(i), z_P(i+1)], '--.k')
%     plot3(theta_P(i), z_P(i), E_CoM(i),'.b')
    txt = num2str(poincare_step_no(i));
    text(theta_P(i)+0.00001, z_P(i) - 0.000001, txt)  
end

% title('Poincare Map');
xlabel('\theta_{P, VLO}');
ylabel('y_{VLO}');
zlabel('Energy of CoM');
grid on
view(0,90)
% xlim([0.2578, 0.2592])
% ylim([0.96646, 0.96653])

% set(gcf, 'color', 'none');
% % set(gca, 'color', 'none');
% export_fig poincare_zoomin.png -m3

end