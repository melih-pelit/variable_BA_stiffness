function trackingPlots(simout, des_z_dz_dx, CoM_acc, sw_ft_pos, sw_ft_des, flag, time)
figure

subplot(3,2,1);
plot(time, des_z_dz_dx(:,1)); hold on;
plot(time, CoM_acc(:,1)); title('x');
plot(time, flag(:,1))
ylim([0,2]); grid on;

subplot(3,2,2);
plot(time, des_z_dz_dx(:,3)); hold on;
plot(time, CoM_acc(:,3)); title('dx');
plot(time, flag(:,1))
ylim([0,2]); grid on;

subplot(3,2,3);
plot(time, des_z_dz_dx(:,2)); hold on;
plot(time, CoM_acc(:,2)); title('z');
plot(time, flag(:,1))
legend('z_{des}','z')
ylim([0.5,1.5]); grid on;

subplot(3,2,4);
plot(time, des_z_dz_dx(:,4)); hold on;
plot(time, CoM_acc(:,4)); title('dz');
plot(time, flag(:,1))
ylim([-3,3]); grid on;


subplot(3,2,5:6);
plot(time, pi/2*ones(length(time),1)); hold on;
trunk_angle = simout(:,1) + simout(:,2) + simout(:,5);
plot(time, trunk_angle);
title('trunk angle')

% swing foot
figure
subplot(2,2,1);
plot(time, sw_ft_des(:,1)); hold on;
plot(time, sw_ft_pos(:,1)); title('swing foot x');
plot(time, flag(:,1))
ylim([-0.1,5]); grid on;

subplot(2,2,2);
plot(time, sw_ft_des(:,2)); hold on;
plot(time, sw_ft_pos(:,2)); title('swing foot z');
plot(time, 0.2*flag(:,1))
ylim([-0.1,0.5]); grid on;

subplot(2,2,3);
plot(time, sw_ft_des(:,3)); hold on;
plot(time, sw_ft_pos(:,3)); title('swing foot dx'); hold on;
plot(time, flag(:,1))
% ylim([-5,5]); 
grid on;

subplot(2,2,4);
plot(time, sw_ft_des(:,4)); hold on;
plot(time, sw_ft_pos(:,4)); title('swing foot dz'); hold on;
plot(time, flag(:,1))
ylim([-5,5]); grid on;

%% plots with ylim 45-50 secs
% figure
% subplot(2,2,1);
% plot(time, des_z_dz_dx(:,1)); hold on;
% plot(time, CoM_acc(:,2)); title('z');
% plot(time, flag(:,1))
% legend('z_{des}','z')
% ylim([0.85,1.1]); grid on;
% xlim([45,50]);
% 
% subplot(2,2,2);
% plot(time, des_z_dz_dx(:,2)); hold on;
% plot(time, CoM_acc(:,4)); title('dz');
% plot(time, flag(:,1))
% ylim([-1.5,1.5]); grid on;
% xlim([45,50]);
% 
% subplot(2,2,3);
% plot(time, des_z_dz_dx(:,3)); hold on;
% plot(time, CoM_acc(:,3)); title('dx');
% plot(time, flag(:,1))
% ylim([0.9,1.5]); grid on;
% xlim([45,50]);
% 
% subplot(2,2,4);
% plot(time, pi/2*ones(length(time),1)); hold on;
% trunk_angle = simout(:,1) + simout(:,2) + simout(:,5);
% plot(time, trunk_angle);
% title('trunk angle')
% xlim([45,50]);
% 
% % swing foot
% figure
% subplot(2,2,1);
% plot(time, sw_ft_des(:,1) - flag(:,2)); hold on;
% plot(time, sw_ft_pos(:,1) - flag(:,2)); title('swing foot x');
% plot(time, flag(:,1))
% ylim([-0.5,0.5]); grid on;
% xlim([45,50]);
% 
% subplot(2,2,2);
% plot(time, sw_ft_des(:,2)); hold on;
% plot(time, sw_ft_pos(:,2)); title('swing foot z');
% plot(time, flag(:,1))
% ylim([-0.02,0.1]); grid on;
% xlim([45,50]);
% 
% subplot(2,2,3);
% plot(time, sw_ft_des(:,3)); hold on;
% plot(time, sw_ft_pos(:,3)); title('swing foot dx'); hold on;
% plot(time, flag(:,1))
% ylim([0.9,4.5]); grid on;
% xlim([45,50]);
% 
% subplot(2,2,4);
% plot(time, sw_ft_des(:,4)); hold on;
% plot(time, sw_ft_pos(:,4)); title('swing foot dz'); hold on;
% plot(time, flag(:,1))
% ylim([-2,1.5]); grid on;
% xlim([45,50]);
end