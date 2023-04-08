function snap_shots(sol)
%%%%% Model Parameters %%%%%
m1 = mean(sol.parameters.m1.value); % mass of link 1 [kg]
m2 = mean(sol.parameters.m2.value); % mass of link 2 [kg]
m3 = mean(sol.parameters.m2.value); % mass of link 3 [kg]
m4 = mean(sol.parameters.m1.value); % mass of link 4 [kg]
m5 = mean(sol.parameters.m5.value); % mass of link 5 (trunk) [kg]

l1 = mean(sol.parameters.l1.value); % length of link n [m]
l2 = mean(sol.parameters.l2.value);
l3 = mean(sol.parameters.l2.value);
l4 = mean(sol.parameters.l1.value);
l5 = mean(sol.parameters.l5.value);

I1 = mean(sol.parameters.I1.value); % rotational inertia [kg m^2]
I2 = mean(sol.parameters.I2.value);
I3 = mean(sol.parameters.I2.value);
I4 = mean(sol.parameters.I1.value);
I5 = mean(sol.parameters.I5.value);

g = mean(sol.parameters.g.value); % gravitational acceleration [m/s^2]

simout = [sol.states.th1.value; sol.states.th2.value; sol.states.th3.value; sol.states.th4.value; sol.states.th5.value; sol.states.dth1.value; sol.states.dth2.value; sol.states.dth3.value; sol.states.dth4.value; sol.states.dth5.value]';
u = [sol.controls.u2.value; sol.controls.u3.value; sol.controls.u4.value; sol.controls.u5.value]';
time = sol.states.time.value;

figure
nt = length(simout);
k = 1;

for i = 1:nt
    current_X = simout(i,:);
    
    th1 = current_X(1);
    th2 = current_X(2);
    th3 = current_X(3);
    th4 = current_X(4);
    th5 = current_X(5);
    
    extra = 0;
    
    x1 = extra + 0.5*k;
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
    
    line_wid = 2;
    plot(V_body(1,:),V_body(2,:), 'k', 'LineWidth', line_wid);
    %     title(['Time = ',num2str(time_t,2)])
    
    %     xlim([-2 + P_hip(1), 2 + P_hip(1)])
    
    ylim([-0.2, 1.5])
%     title(['t = ' num2str(i*sample_time)])
    hold on
    plot(V_links1(1,:),V_links1(2,:), 'r', 'LineWidth', line_wid);
    plot(V_links2(1,:),V_links2(2,:), 'r', 'LineWidth', line_wid);
    plot(V_linkw1(1,:),V_linkw1(2,:), 'b', 'LineWidth', line_wid);
    plot(V_linkw2(1,:),V_linkw2(2,:), 'b', 'LineWidth', line_wid);
    plot([-3 + x1 3 + x1],[0 0],'k', 'LineWidth', 1) % ground line
    
    xlim([0, 7])
    k = k + 1;
%     pause
%     hold off
    
end

pbaspect([2 1 1])
set(gcf, 'color', 'none');
set(gca, 'color', 'none');
export_fig snap_shots.png -m3
end