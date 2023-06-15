function snap_shots(simout, param, flag)

f_print = 1;

%%%%% Model Parameters %%%%%
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
%%%%%%%%%%%


figure(3)
nt = length(simout);
k = 1;

frame_start = 12997;
frame_end = 14177;

frame_leap = 95;
x1 = 0; %initial location of the stance foot
flag_ds = 0; % flag used to detect when ds change happens so we can add an offset to x1

for i = frame_start:frame_leap:frame_end
    current_X = simout(i,:);
    
    th1 = current_X(1);
    th2 = current_X(2);
    th3 = current_X(3);
    th4 = current_X(4);
    th5 = current_X(5);
    
    dth1 = current_X(6);
    dth2 = current_X(7);
    dth3 = current_X(8);
    dth4 = current_X(9);
    dth5 = current_X(10);
    
    cur_flag = flag(i,1);
    
    if cur_flag == -1 && flag_ds == 0
        x1 = x1 + 1;
        flag_ds = 1;
    else
        x1 = x1 + 0.55;
    end
    
%     x1 = extra + 1*k;
    z1 = 0;
    
    % Joint Locations
    Pb = [ x1 + l1*cos(th1) + l2*cos(th1 + th2) + l5*cos(th1 + th2 + th5);
        z1 + l1*sin(th1) + l2*sin(th1 + th2) + l5*sin(th1 + th2 + th5)];
    Ps1 = [ x1 + l1*cos(th1);
        z1 + l1*sin(th1)];
    Pw1 = [ x1 + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3);
        z1 + l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)];
    Pst_tip = [ x1;
        z1];
    Psw_tip = [ x1 + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + l4*cos(th1 + th2 + th3 + th4);
        z1 + l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + l4*sin(th1 + th2 + th3 + th4)];
    P_hip = [x1 + l1*cos(th1) + l2*cos(th1 + th2);
        z1 + l1*sin(th1) + l2*sin(th1 + th2)];
    % ƒŠƒ“ƒN‚Ì•`‰æ
    V_body = [x1 + l1*cos(th1) + l2*cos(th1 + th2), Pb(1);z1 + l1*sin(th1) + l2*sin(th1 + th2), Pb(2)];
    V_links1 = [x1 Ps1(1);z1 Ps1(2)];
    V_links2 = [Ps1(1) P_hip(1);Ps1(2) P_hip(2)];
    V_linkw1 = [P_hip(1) Pw1(1);P_hip(2) Pw1(2)];
    V_linkw2 = [Pw1(1) Psw_tip(1);Pw1(2) Psw_tip(2)];
    
    line_wid = 3;
    
    %%%%% drawing the body %%%%%
    body_mid = [(V_body(1,1) + V_body(1,end))/2; (V_body(2,1) + V_body(2,end))/2];
    body_angle = atan2(V_body(2,end) - V_body(2,1), V_body(1,end) - V_body(1,1));
    body_angle = rad2deg(body_angle);
    body_height = sqrt((V_body(2,end) - V_body(2,1))^2 + (V_body(1,end) - V_body(1,1))^2)
    rectangle2([body_mid(1) body_mid(2) body_height, 0.12], 'Curvature', .2, 'Rotation', body_angle,'LineWidth',1.5,'FaceColor', [0.5882    0.5882    0.5882]); % [x_center, y_center, width, height]
    
    % plot(V_body(1,:),V_body(2,:), 'k', 'LineWidth', 15);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %     xlim([-2 + P_hip(1), 2 + P_hip(1)])
    
    ylim([-0.2, 1.5])
    %     title(['t = ' num2str(i*sample_time)])
    hold on
    
    plot(V_links1(1,:),V_links1(2,:), 'r', 'LineWidth', line_wid);
    plot(V_links2(1,:),V_links2(2,:), 'r', 'LineWidth', line_wid);
    plot(Ps1(1), Ps1(2), 'r.', 'MarkerSize', 15) % plotting the stance knee
    plot(V_linkw1(1,:),V_linkw1(2,:), 'b', 'LineWidth', line_wid);
    plot(V_linkw2(1,:),V_linkw2(2,:), 'b', 'LineWidth', line_wid);
    plot(Pw1(1), Pw1(2), 'b.', 'MarkerSize', 15) % plotting the swing knee
   
    
    
    xlim([-0.1, 9])
    com(k,:) = calculate_com(current_X, param, x1, [0;0;0;0;0]);
    k = k + 1;
%     pause
%     hold off
    
end

plot([-20 + x1 3 + x1],[0 0],'k', 'LineWidth', 2) % ground line
% plot(com(:,1), com(:,2), '-.', 'Color',[0.8500, 0.3250, 0.0980] , 'LineWidth', 1.5)
pbaspect([0.8*(8+0.1)/(1.5+0.2) 1 1])

if f_print == 1
    set(gcf, 'color', 'none');
    set(gca, 'color', 'none');
end

end