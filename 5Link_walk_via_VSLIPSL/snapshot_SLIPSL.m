function snapshot_SLIPSL(dc)
% 2020.06.30 - function to create snap shots of SLIPSL walking

% ----------- parameters ----------------------
L0 = dc.param.L0; % [m]
k0 = dc.param.k0; % nominal leg stiffness [N/m]
m_CoM = dc.param.m_CoM; % hip mass [kg]
m_swLeg = dc.param.m_swLeg; % [kg]
m_swFoot = dc.param.m_swFoot; % [kg]
L_thigh = dc.param.L_thigh;

simout_ss = [dc.x(1:13)', dc.z(1:13)', dc.theta(1:13)', dc.r_swFoot(1:13)'];
simout_ds = [dc.x(14:26)', dc.z(14:26)', dc.theta(14:26)', dc.r_swFoot(14:26)'];

% spring animation parameters
coilNo = 10;
springrad = 0.2;
a00 = 0.05;
springLW = 3;

% snapshots of single stance phase

x_CoM = -0.3; % initial x_CoM value
x_CoM_inc = 1; % x_CoM increment value

figure
hold on
plot([-20, 20],[0 0],'k', 'LineWidth', 2) % ground line
line_width = 2.5;

% plotting the ground line

for i=2:1:(dc.N_ss+1)
    
    X = simout_ss(i,:);
    
    % variables
    x_CoM = x_CoM + x_CoM_inc;
    z_CoM = X(2);
    theta = X(3);
    r_swFoot = X(4);
    
    foot = (dc.soln.foot - X(1)) + x_CoM;
    
    hip = [x_CoM; z_CoM];
    knee = [x_CoM + L_thigh*cos(theta); z_CoM - L_thigh*sin(theta)];
    foot_sw = [x_CoM + (L_thigh + r_swFoot)*cos(theta); z_CoM-(L_thigh + r_swFoot)*sin(theta)];
    
    % plotting the linear spring
    [x_swft, y_swft] = spring(knee(1), knee(2), foot_sw(1), foot_sw(2), 5, springrad, a00);
    plot(x_swft, y_swft, 'k', 'LineWidth', line_width)
    
    plot([hip(1), knee(1)], [hip(2), knee(2)], 'b', 'LineWidth', line_width);
    plot(foot_sw(1), foot_sw(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    %         plot([hip(1), foot], [hip(2), 0], 'g', 'LineWidth', 2) % plotting the stance leg
    st_vector = [x_CoM - foot, z_CoM]; % stance leg vector from CoM to foot
    plot([x_CoM, x_CoM - 0.2*st_vector(1)], [z_CoM, z_CoM - 0.2*st_vector(2)], 'r', 'LineWidth', line_width);
    %         hold on
    plot([foot, x_CoM - 0.8*st_vector(1)], [0, z_CoM - 0.8*st_vector(2)], 'r', 'LineWidth', line_width);
    
    [x_st, y_st] = spring(x_CoM - 0.2*st_vector(1), z_CoM - 0.2*st_vector(2), x_CoM - 0.8*st_vector(1), z_CoM - 0.8*st_vector(2), coilNo, springrad, a00);
    plot(x_st, y_st, 'r', 'LineWidth', springLW)
    
    plot(hip(1), hip(2), 'ko', 'MarkerSize', 16, 'MarkerFaceColor', 'k');
    
end
% snapshots of double stance phase

for i = 1:3:(dc.N_ds + 1)
    
    X = simout_ds(i,:);
    
    % variables
    x_CoM = x_CoM + x_CoM_inc;
    z_CoM = X(2);
    theta = X(3);
    r_swFoot = X(4);
    
    hip = [x_CoM; z_CoM];
    knee = [x_CoM + L_thigh*cos(theta); z_CoM - L_thigh*sin(theta)];
    foot_sw = [x_CoM + (L_thigh + r_swFoot)*cos(theta); z_CoM-(L_thigh + r_swFoot)*sin(theta)];
    
    
    foot = (dc.soln.footPlus - X(1)) + x_CoM;
    foot_prev = (dc.soln.foot - X(1)) + x_CoM;
    
    % plotting the stance leg
    st_vector = [x_CoM - foot, z_CoM];
    plot([x_CoM, x_CoM - 0.2*st_vector(1)], [z_CoM, z_CoM - 0.2*st_vector(2)], 'r', 'LineWidth', line_width);
    %         hold on
    plot([foot, x_CoM - 0.8*st_vector(1)], [0, z_CoM - 0.8*st_vector(2)], 'r', 'LineWidth', line_width);
    
    
    [x_st, y_st] = spring(x_CoM - 0.2*st_vector(1), z_CoM - 0.2*st_vector(2), x_CoM - 0.8*st_vector(1), z_CoM - 0.8*st_vector(2), coilNo, springrad, a00);
    plot(x_st, y_st, 'r', 'LineWidth', springLW)
    
    % plotting the swing leg
    sw_vector = [x_CoM - foot_prev, z_CoM];
    plot([x_CoM, x_CoM - 0.2*sw_vector(1)], [z_CoM, z_CoM - 0.2*sw_vector(2)], 'b', 'LineWidth', line_width);
    plot([foot_prev, x_CoM - 0.8*sw_vector(1)], [0, z_CoM - 0.8*sw_vector(2)], 'b', 'LineWidth', line_width);
    
    [x_sw, y_sw] = spring(x_CoM - 0.2*sw_vector(1), z_CoM - 0.2*sw_vector(2), x_CoM - 0.8*sw_vector(1), z_CoM - 0.8*sw_vector(2), coilNo, springrad, a00);
    plot(x_sw, y_sw, 'b', 'LineWidth', springLW)
    
    plot(hip(1), hip(2), 'ko', 'MarkerSize', 16, 'MarkerFaceColor', 'k');
    
end

xlim([-0.5, 17.5])
ylim ([-0.2, 1.2])
pbaspect([0.2*(17) 1 1])

set(gcf, 'color', 'none');
set(gca, 'color', 'none');

% function to plot the springs
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