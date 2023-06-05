function animation(f_video, simout, sample_time, param, f_pause, frame_leap, flag, step_no, force, des_traj, des_th)
pause
if f_video == 1
    video_v = VideoWriter('5link_SLIP_walking.avi');
    open(video_v);
end
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
start_sec = sample_time; % start second for the animation
end_sec = 15; % ending second for the animation

figure()
nt = length(simout);
axis equal

%SLIP flags
% SLIP_flags = [1, 1, 0];
% ft1 = 0; % setting initial feet locations for SLIP mdeol to 0
% ft2 = 0; % setting initial feet locations for SLIP mdeol to 0
%
% SLIP_params = [L_slip, alpha0, k_slip, m_slip, g];

% for i = start_sec/sample_time:frame_leap:end_sec/sample_time
for i = start_sec/sample_time:frame_leap:end_sec/sample_time
    
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
    
    x1 = flag(i,2);
    z1 = 0;
    
        %%%% ------ Plotting desired joint angles ---------------------------%
    % desired Joint Locations
    des_th1 = des_th(i,1);
    des_th2 = des_th(i,2);
    des_th3 = des_th(i,3);
    des_th4 = des_th(i,4);
    des_th5 = des_th(i,5);
    
    Pb_des = [ x1 + l1*cos(des_th1) + l2*cos(des_th1 + des_th2) + l5*cos(des_th1 + des_th2 + des_th5);
        z1 + l1*sin(des_th1) + l2*sin(des_th1 + des_th2) + l5*sin(des_th1 + des_th2 + des_th5)];
    Ps1_des = [ x1 + l1*cos(des_th1);
        z1 + l1*sin(des_th1)];
    Pw1_des = [ x1 + l1*cos(des_th1) + l2*cos(des_th1 + des_th2) + l3*cos(des_th1 + des_th2 + des_th3);
        z1 + l1*sin(des_th1) + l2*sin(des_th1 + des_th2) + l3*sin(des_th1 + des_th2 + des_th3)];
    Pst_tip_des = [ x1;
        z1];
    Psw_tip_des = [ x1 + l1*cos(des_th1) + l2*cos(des_th1 + des_th2) + l3*cos(des_th1 + des_th2 + des_th3) + l4*cos(des_th1 + des_th2 + des_th3 + des_th4);
        z1 + l1*sin(des_th1) + l2*sin(des_th1 + des_th2) + l3*sin(des_th1 + des_th2 + des_th3) + l4*sin(des_th1 + des_th2 + des_th3 + des_th4)];
    P_hip_des = [x1 + l1*cos(des_th1) + l2*cos(des_th1 + des_th2);
        z1 + l1*sin(des_th1) + l2*sin(des_th1 + des_th2)];
    % ƒŠƒ“ƒN‚Ì•`‰æ
    V_body_des = [x1 + l1*cos(des_th1) + l2*cos(des_th1 + des_th2), Pb_des(1);z1 + l1*sin(des_th1) + l2*sin(des_th1 + des_th2), Pb_des(2)];
    V_links1_des = [x1 Ps1_des(1);z1 Ps1_des(2)];
    V_links2_des = [Ps1_des(1) P_hip_des(1);Ps1_des(2) P_hip_des(2)];
    V_linkw1_des = [P_hip_des(1) Pw1_des(1);P_hip_des(2) Pw1_des(2)];
    V_linkw2_des = [Pw1_des(1) Psw_tip_des(1);Pw1_des(2) Psw_tip_des(2)];
    
    gray_color = 150;
    des_plot_width = 6;
    
    p1 = plot(V_body_des(1,:),V_body_des(2,:), 'Color', [gray_color gray_color gray_color]/255, 'LineWidth', des_plot_width);
    %     title(['Time = ',num2str(time_t,2)])
    hold on
    
    p2 = plot(V_links1_des(1,:),V_links1_des(2,:), 'Color', [gray_color gray_color gray_color]/255, 'LineWidth', des_plot_width);
    p3 = plot(V_links2_des(1,:),V_links2_des(2,:), 'Color', [gray_color gray_color gray_color]/255, 'LineWidth', des_plot_width);
    p4 = plot(V_linkw1_des(1,:),V_linkw1_des(2,:), 'Color', [gray_color gray_color gray_color]/255, 'LineWidth', des_plot_width);
    p5 = plot(V_linkw2_des(1,:),V_linkw2_des(2,:), 'Color', [gray_color gray_color gray_color]/255, 'LineWidth', des_plot_width);
    
%     p1.Color(4) = 0.6;
%     p2.Color(4) = 0.6;
%     p3.Color(4) = 0.6;
%     p4.Color(4) = 0.6;
%     p5.Color(4) = 0.6;
    
    CoM_des = calculate_com(des_th(i,:), param, x1, [0;0;0;0;0]);
    plot(CoM_des(1), CoM_des(2), 'o', 'MarkerSize', 2, 'MarkerFaceColor', 'g');
    % -------------------------------------------------------------------%
    
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
    
    plot(V_body(1,:),V_body(2,:), 'k', 'LineWidth', 2);
    %     title(['Time = ',num2str(time_t,2)])
    
    %     xlim([-2 + P_hip(1), 2 + P_hip(1)])
    xlim([-1 + P_hip(1), 1 + P_hip(1)])
    ylim([-0.5, 1.5])
    
    
    title(['t = ' num2str(i*sample_time) ' [sec], F_{ss} = ', num2str(flag(i,1))])
    grid on
    hold on
    plot(V_links1(1,:),V_links1(2,:), 'r', 'LineWidth', 2);
    plot(V_links2(1,:),V_links2(2,:), 'r', 'LineWidth', 2);
    plot(V_linkw1(1,:),V_linkw1(2,:), 'k', 'LineWidth', 2);
    plot(V_linkw2(1,:),V_linkw2(2,:), 'k', 'LineWidth', 2);
    plot([-3 + x1 3 + x1],[0 0],'k') % ground line
    
    % plotting CoM----------------------------------------------
    CoM= calculate_com(current_X, param, x1, [0;0;0;0;0]);
    xG = CoM(1);
    zG = CoM(2);
    plot(xG, zG, 'o', 'MarkerSize', 7, 'MarkerFaceColor', 'b');
    %-----------------------------------------------------------
    
    % plotting desired location of swing foot-------------------
    plot(des_traj(i, 3), des_traj(i, 4), 'o', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
    %-----------------------------------------------------------
    
    %---- plotting des CoM ------------------------------------
    des_x_CoM = des_traj(i,1);
    des_y_CoM = des_traj(i,2);
    plot(des_x_CoM, des_y_CoM, 'o', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
    %----------------------------------------------------------
    
    %txt1 = ['f_{SS}: ' num2str(flag(i,1))]; 
    %txt2 = ['step no: ' num2str(step_no(i))]; 
    %txt3 = ['constraint force (z): ' num2str(force(i,2))]; 
    %text(xG - 0.5, 0.8,txt1)
    %text(xG - 0.5, 0.6,txt2)
    %text(xG - 0.5, 0.4,txt3)
    % -----------------------------------------------------
    
    %---- plotting the corresponding SLISL ------------------------------%
%     if flag(i,1) == 1
%         % if the system is in the SS phase
%         [x_M(i), y_M(i), theta(i), r(i)] = calc_5linkTOslipsl(simout(i,:), flag(i,:), param, slipslParams);
%         L_thigh = slipslParams(4);
%         
%         plot([x1, x1 + x_M(i)], [0, y_M(i)], 'g')
%         plot([x1 + x_M(i), x1 + x_M(i)+L_thigh*cos(theta(i))], [y_M(i), y_M(i)+L_thigh*sin(theta(i))], 'g')
%         plot([x1 + x_M(i), x1 + x_M(i)+L_thigh*cos(theta(i))], [y_M(i), y_M(i)+L_thigh*sin(theta(i))], 'g')
%         
%         plot(x1 + x_M(i), y_M(i), '.g', 'MarkerSize', 30)
%         plot(x1 + x_M(i)+(L_thigh+r(i))*cos(theta(i)), y_M(i)+(L_thigh+r(i))*sin(theta(i)), '.g', 'MarkerSize', 30)
%         
%         
%     else
%         % if the system is in the DS phase        
%       
%     end
    % -------------------------------------------------------------------%
    

    
    hold off
    
    xlim([-1.4 + P_hip(1), 1.4 + P_hip(1)])
    ylim([-0.5, 1.5])
    
    if f_video == 1
        frame = getframe(gcf);
        writeVideo(video_v,frame);
    end
    
    if f_pause == 1
        pause
    else
        pause(0.01)
    end
    
end
end