function animation(f_video, sol, f_pause)

if f_pause == 1
    pause
else
    pause(0.3)
end

if f_video == 1
    video_v = VideoWriter('5link_SLIP_walking.avi');
    open(video_v);
end

%%%%% Model Parameters %%%%%
m1 = sol{1}.parameters.m1(1).value;
m2 = sol{1}.parameters.m2(1).value;
m3 = m2;
m4 = m1;
m5 = sol{1}.parameters.m5(1).value;

l1 = sol{1}.parameters.l1(1).value;
l2 = sol{1}.parameters.l2(1).value;
l3 = l2;
l4 = l1;
l5 = sol{1}.parameters.l5(1).value;
%%%%%%%%%%%

figure()
x_st = [0];

for j = 1:length(sol)
    nt = length(sol{j}.states.time.value);
    time = sol{j}.states.time.value;
    simout = [
        sol{j}.states.th1.value', ...
        sol{j}.states.th2.value', ...
        sol{j}.states.th3.value', ...
        sol{j}.states.th4.value', ...
        sol{j}.states.th5.value'];

    for i = 1:nt
        
        current_X = simout(i,:);
        
        th1 = current_X(1);
        th2 = current_X(2);
        th3 = current_X(3);
        th4 = current_X(4);
        th5 = current_X(5);
        
        x1 = x_st(j);
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
        title(['t = ' num2str(time(i))])
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
        
        % is SS phase, calculate the touch-down loc.
        if j == 1 && i == nt
            x_st(end+1) = Psw_tip(1);                
        end
        
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

display(x_st)

end % function end