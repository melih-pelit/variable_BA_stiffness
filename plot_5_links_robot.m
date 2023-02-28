function plot_5_links_robot(state, x_stance_foot, y_stance_foot, param, f_ba_springs, p)

%%% This function plots the 5 links robot

%% set default plot parameters

if nargin < 6
    st_leg_color = 'r';
    sw_leg_color = 'b';
    body_color = 'k';
else
    st_leg_color = p.st_leg_color;
    sw_leg_color = p.sw_leg_color;
    body_color = p.body_color;
end
%% robot parameters

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

% biarticular parameters
r_k = param(11)*1;
r_h = param(12)*1;
k_ba = param(13);
phi_h0 = param(14);
phi_k0 = param(15);

% robot state
th1 = state(1);
th2 = state(2);
th3 = state(3);
th4 = state(4);
th5 = state(5);

% Joint Locations
Pb = [ x_stance_foot + l1*cos(th1) + l2*cos(th1 + th2) + l5*cos(th1 + th2 + th5);
    y_stance_foot + l1*sin(th1) + l2*sin(th1 + th2) + l5*sin(th1 + th2 + th5)];
Ps1 = [ x_stance_foot + l1*cos(th1);
    y_stance_foot + l1*sin(th1)];
Pw1 = [ x_stance_foot + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3);
    y_stance_foot + l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)];
Pst_tip = [ x_stance_foot;
    y_stance_foot];
Psw_tip = [ x_stance_foot + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + l4*cos(th1 + th2 + th3 + th4);
    y_stance_foot + l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + l4*sin(th1 + th2 + th3 + th4)];
P_hip = [x_stance_foot + l1*cos(th1) + l2*cos(th1 + th2);
    y_stance_foot + l1*sin(th1) + l2*sin(th1 + th2)];
% ƒŠƒ“ƒN‚Ì•`‰æ
V_body = [x_stance_foot + l1*cos(th1) + l2*cos(th1 + th2), Pb(1);y_stance_foot + l1*sin(th1) + l2*sin(th1 + th2), Pb(2)];
V_links1 = [x_stance_foot Ps1(1);y_stance_foot Ps1(2)];
V_links2 = [Ps1(1) P_hip(1);Ps1(2) P_hip(2)];
V_linkw1 = [P_hip(1) Pw1(1);P_hip(2) Pw1(2)];
V_linkw2 = [Pw1(1) Psw_tip(1);Pw1(2) Psw_tip(2)];

line_wid = 2.5;
plot(V_body(1,:),V_body(2,:), 'Color', body_color, 'LineWidth', line_wid);
hold on
plot(V_links1(1,:),V_links1(2,:), 'Color', st_leg_color, 'LineWidth', line_wid);
plot(V_links2(1,:),V_links2(2,:), 'Color', st_leg_color, 'LineWidth', line_wid);
plot(V_linkw1(1,:),V_linkw1(2,:), 'Color', sw_leg_color, 'LineWidth', line_wid);
plot(V_linkw2(1,:),V_linkw2(2,:), 'Color', sw_leg_color, 'LineWidth', line_wid);
hold off
%% plotting BA springs
if f_ba_springs == 1
    hold on
    
    % plotting pulleys in hip and knee
    x_knee_st = x_stance_foot + l1*cos(th1);
    y_knee_st = y_stance_foot + l1*sin(th1);
    x_hip = x_stance_foot + l1*cos(th1) + l2*cos(th1 + th2);
    y_hip = y_stance_foot + l1*sin(th1) + l2*sin(th1 + th2);
    x_knee_sw = x_stance_foot + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3);
    y_knee_sw = y_stance_foot + l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3);
    
    %%%%% plotting the biarticular springs
    k1 = [x_stance_foot + l1*cos(th1) + r_k*cos(th1+3*pi/2), y_stance_foot + l1*sin(th1) + r_k*sin(th1+3*pi/2)]; % spring positions on stance knee
    k2 = [x_stance_foot + l1*cos(th1) + r_k*cos(th1+pi/2), y_stance_foot + l1*sin(th1) + r_k*sin(th1+pi/2)]; % spring positions on stance knee
    h1 = [x_stance_foot + l1*cos(th1) + l2*cos(th1+th2) + r_h*cos(th1+th2+3*pi/2), y_stance_foot + l1*sin(th1) + l2*sin(th1+th2) + r_h*sin(th1+th2+3*pi/2)]; % position of RF on stance leg hip
    h2 = [x_stance_foot + l1*cos(th1) + l2*cos(th1+th2) + r_h*cos(th1+th2+pi/2), y_stance_foot + l1*sin(th1) + l2*sin(th1+th2) + r_h*sin(th1+th2+pi/2)]; % position of HA on stance leg hip
    
    k1_sw = [x_stance_foot + l1*cos(th1) + l2*cos(th1+th2) + l3*cos(th1+th2+th3) + r_k*cos(th1+th2+th3+pi/2), y_stance_foot + l1*sin(th1) + l2*sin(th1+th2) + l3*sin(th1+th2+th3) + r_k*sin(th1+th2+th3+pi/2)]; % spring positions on swing knee
    k2_sw = [x_stance_foot + l1*cos(th1) + l2*cos(th1+th2) + l3*cos(th1+th2+th3) + r_k*cos(th1+th2+th3+3*pi/2), y_stance_foot + l1*sin(th1) + l2*sin(th1+th2) + l3*sin(th1+th2+th3) + r_k*sin(th1+th2+th3+3*pi/2)];
    
    RF_1 = h1 + (1/4).*[k1(1) - h1(1), k1(2) - h1(2)];
    RF_2 = k1 + (1/4).*[h1(1) - k1(1), h1(2) - k1(2)];
    
    HA_1 = h2 + (1/4).*[k2(1) - h2(1), k2(2) - h2(2)];
    HA_2 = k2 + (1/4).*[h2(1) - k2(1), h2(2) - k2(2)];
    
    RF_1_sw = h1 + (1/4).*[k1_sw(1) - h1(1), k1_sw(2) - h1(2)];
    RF_2_sw = k1_sw + (1/4).*[h1(1) - k1_sw(1), h1(2) - k1_sw(2)];
    
    HA_1_sw = h2 + (1/4).*[k2_sw(1) - h2(1), k2_sw(2) - h2(2)];
    HA_2_sw = k2_sw + (1/4).*[h2(1) - k2_sw(1), h2(2) - k2_sw(2)];
    
    coilNo = 5;
    springrad = 0.4;
    a00 = 0.01;
    
    [x_RF_st,y_RF_st] = spring(RF_2(1), RF_2(2), RF_1(1), RF_1(2), coilNo, springrad, a00);
    [x_HA_st,y_HA_st] = spring(HA_2(1), HA_2(2), HA_1(1), HA_1(2), coilNo, springrad, a00);
    
    [x_RF_sw,y_RF_sw] = spring(RF_2_sw(1), RF_2_sw(2), RF_1_sw(1), RF_1_sw(2), coilNo, springrad, a00);
    [x_HA_sw,y_HA_sw] = spring(HA_2_sw(1), HA_2_sw(2), HA_1_sw(1), HA_1_sw(2), coilNo, springrad, a00);
    
    
    springLW = 1;
    springColor = 'k';
    
    % stance leg springs
    plot(x_RF_st,y_RF_st, springColor,'LineWidth',springLW)
    plot([h1(1), RF_1(1)], [h1(2), RF_1(2)], springColor,'LineWidth',springLW)
    plot([k1(1), RF_2(1)], [k1(2), RF_2(2)], springColor,'LineWidth',springLW)
    
    plot(x_HA_st,y_HA_st, springColor,'LineWidth',springLW)
    plot([h2(1), HA_1(1)], [h2(2), HA_1(2)], springColor,'LineWidth',springLW)
    plot([k2(1), HA_2(1)], [k2(2), HA_2(2)], springColor,'LineWidth',springLW)
    
    % swing leg springs
    plot(x_RF_sw,y_RF_sw, springColor,'LineWidth',springLW)
    plot([h1(1), RF_1_sw(1)], [h1(2), RF_1_sw(2)], springColor,'LineWidth',springLW)
    plot([k1_sw(1), RF_2_sw(1)], [k1_sw(2), RF_2_sw(2)], springColor,'LineWidth',springLW)
    
    plot(x_HA_sw,y_HA_sw, springColor,'LineWidth',springLW)
    plot([h2(1), HA_1_sw(1)], [h2(2), HA_1_sw(2)], springColor,'LineWidth',springLW)
    plot([k2_sw(1), HA_2_sw(1)], [k2_sw(2), HA_2_sw(2)], springColor,'LineWidth',springLW)
    %%%%%%
    
    % plotting the pulleys
    rectangle('Position',[x_knee_st - r_k, y_knee_st - r_k, 2*r_k, 2*r_k],'Curvature',[1 1], 'FaceColor', 'r')
    rectangle('Position',[x_knee_sw - r_k, y_knee_sw - r_k, 2*r_k, 2*r_k],'Curvature',[1 1], 'FaceColor', 'r')
    rectangle('Position',[x_hip - r_h, y_hip - r_h, 2*r_h, 2*r_h],'Curvature',[1 1], 'FaceColor', 'r')
    hold off
end

    function [xs ys] = spring(xa,ya,xb,yb,varargin)
        % SPRING         Calculates the position of a 2D spring
        %    [XS YS] = SPRING(XA,YA,XB,YB,NE,A,R0) calculates the position of
        %    points XS, YS of a spring with ends in (XA,YA) and (XB,YB), number
        %    of coils equal to NE, natural length A, and natural radius R0.
        %    Useful for mass-spring oscillation animations.
        % USAGE: in a first call in your code, call it with the full parameters.
        % Then, only you have to give it the coordinates of the ends.
        % EXAMPLE:
        % xa = 0; ya = 0; xb = 2; yb = 2; ne = 10; a = 1; ro = 0.1;
        % [xs,ys] = spring(xa,ya,xb,yb,ne,a,ro); plot(xs,ys,'LineWidth',2)
        %...
        % [xs,ys]=spring(xa,ya,xb,yb); plot(xs,ys,'LineWidth',2)
        %
        %   Made by:            Gustavo Morales   UC  08-17-09 gmorales@uc.edu.ve
        %
        persistent ne Li_2 ei b
        if nargin > 4 % calculating some fixed spring parameters only once time
            [ne a r0] = varargin{1:3};                  % ne: number of coils - a = natural length - r0 = natural radius
            Li_2 =  (a/(4*ne))^2 + r0^2;                % (large of a quarter of coil)^2
            ei = 0:(2*ne+1);                            % vector of longitudinal positions
            j = 0:2*ne-1; b = [0 (-ones(1,2*ne)).^j 0]; % vector of transversal positions
        end
        R = [xb yb] - [xa ya]; mod_R = norm(R); % relative position between "end_B" and "end_A"
        L_2 = (mod_R/(4*ne))^2; % (actual longitudinal extensión of a coil )^2
        if L_2 > Li_2
            error('Spring:TooEnlargement', ...
                'Initial conditions cause pulling the spring beyond its maximum large. \n Try reducing these conditions.')
        else
            r = sqrt(Li_2 - L_2);   %actual radius
        end
        c = r*b;    % vector of transversal positions
        u1 = R/mod_R; u2 = [-u1(2) u1(1)]; % unitary longitudinal and transversal vectors
        xs = xa + u1(1)*(mod_R/(2*ne+1)).*ei + u2(1)*c; % horizontal coordinates
        ys = ya + u1(2)*(mod_R/(2*ne+1)).*ei + u2(2)*c; % vertical coordinates
    end
end