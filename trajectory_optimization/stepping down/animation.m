function animation(f_video, simout, time, param, f_pause)
pause
if f_video == 1
    video_v = VideoWriter('5link_SLIP_walking.avi');
    open(video_v);
end

%%%%% Model Parameters %%%%%
m1 = param.m1;
m2 = param.m2;
m3 = m2;
m4 = m1;
m5 = param.m5;

l1 = param.l1;
l2 = param.l2;
l3 = param.l3;
l4 = param.l4;
l5 = param.l5;
%%%%%%%%%%%
start_sec = 0; % start second for the animation
end_sec = 20; % ending second for the animation

% figure(3)
figure('units','pixels','position',[0 0 720 720])
nt = length(simout);

for i = 1:nt
    
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
    
    x1 = 0;
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
    
    plot(V_body(1,:),V_body(2,:), 'LineWidth', 2);
    %     title(['Time = ',num2str(time_t,2)])
    
    %     xlim([-2 + P_hip(1), 2 + P_hip(1)])
    xlim([-1 + P_hip(1), 1 + P_hip(1)])
    ylim([-0.5, 1.7])
    title(['t = ' num2str(time(i)) ', step = ' num2str(i)])
    grid on
    hold on
    plot(V_links1(1,:),V_links1(2,:), 'r', 'LineWidth', 2);
    plot(V_links2(1,:),V_links2(2,:), 'r', 'LineWidth', 2);
    plot(V_linkw1(1,:),V_linkw1(2,:), 'k', 'LineWidth', 2);
    plot(V_linkw2(1,:),V_linkw2(2,:), 'k', 'LineWidth', 2);
    plot([-3 + x1 3 + x1],[0 0],'k') % ground line
    
    % plotting CoM----------------------------------------------
%     CoM= calculate_com(current_X, param, x1, [0;0;0;0;0]);
%     xG = CoM(1);
%     zG = CoM(2);
%     plot(xG, zG, 'o', 'MarkerSize', 7, 'MarkerFaceColor', 'b');
    
    % plotting desired location of swing foot-------------------
%     plot(sw_ft_des(i, 1), sw_ft_des(i, 2), 'o', 'MarkerSize', 3, 'MarkerFaceColor', 'r');

    hold off
    
    if f_video == 1
        frame = getframe(gcf);
        writeVideo(video_v,frame);
        writeVideo(video_v,frame);
        writeVideo(video_v,frame);
    end
    
    if f_pause == 1
        pause
    else
        pause(0.3)
    end
    
end
end