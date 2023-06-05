function trackingPlots_jointAngles(simout, flag, time, des_th)

%% Calculations
% detect state change
flag_prev = flag(1,1);
state_change_idx = [];
for i = 2:length(flag(:,1))
    flag_cur = flag(i,1);
    if flag_cur ~= flag_prev
        state_change_idx(end + 1) = i;
    end
    flag_prev = flag_cur;
end

% TODO: displace state change

%% plotting

x_start = 0;
x_end = time(end);

figure()

subplot(5,1,1);
plot(time, mod(des_th(:,1),2*pi)); hold on;
plot(time, mod(simout(:,1),2*pi)); title('th1');
% plot(time, flag(:,1))
grid on;
xlim([x_start, x_end])

subplot(5,1,2);
plot(time, mod(des_th(:,2),2*pi)); hold on;
plot(time, mod(simout(:,2),2*pi)); title('th2');
% plot(time, flag(:,1))
grid on;
xlim([x_start, x_end])

subplot(5,1,3);
plot(time, mod(des_th(:,3),2*pi)); hold on;
plot(time, mod(simout(:,3),2*pi)); title('th3');
% plot(time, flag(:,1))
grid on;
xlim([x_start, x_end])

subplot(5,1,4);
plot(time, mod(des_th(:,4),2*pi)); hold on;
plot(time, mod(simout(:,4),2*pi)); title('th4');
% plot(time, flag(:,1))
grid on;
xlim([x_start, x_end])

subplot(5,1,5);
plot(time, mod(des_th(:,5), 2*pi)); hold on;
plot(time, mod(simout(:,5), 2*pi)); title('th5');
% plot(time, flag(:,1))
grid on;
xlim([x_start, x_end])

% plotting velocity tracking
figure()

subplot(5,1,1);
plot(time, des_th(:,6)); hold on;
plot(time, simout(:,6)); title('dth1');
% plot(time, flag(:,1))
grid on;
xlim([x_start, x_end])

subplot(5,1,2);
plot(time, des_th(:,7)); hold on;
plot(time, simout(:,7)); title('dth2');
% plot(time, flag(:,1))
grid on;
xlim([x_start, x_end])

subplot(5,1,3);
plot(time, des_th(:,8)); hold on;
plot(time, simout(:,8)); title('dth3');
% plot(time, flag(:,1))
grid on;
xlim([x_start, x_end])

subplot(5,1,4);
plot(time, des_th(:,9)); hold on;
plot(time, simout(:,9)); title('dth4');
% plot(time, flag(:,1))
grid on;
xlim([x_start, x_end])

subplot(5,1,5);
plot(time, des_th(:,10)); hold on;
plot(time, simout(:,10)); title('dth5');
% plot(time, flag(:,1))
grid on;
xlim([x_start, x_end])


end