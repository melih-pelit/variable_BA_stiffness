function draw_robot(X,param, x_CoM_des, z_CoM_des, x_sw_des, z_sw_des)

%% parameters

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

%% defining variables

th1 = X(1);
th2 = X(2);
th3 = X(3);
th4 = X(4);
th5 = X(5);

x1 = 0;
z1 = 0;

%% 各点の位置?ﾀ標
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
%% リンクの描画
V_body = [x1 + l1*cos(th1) + l2*cos(th1 + th2), Pb(1);z1 + l1*sin(th1) + l2*sin(th1 + th2), Pb(2)];
V_links1 = [x1 Ps1(1);z1 Ps1(2)];
V_links2 = [Ps1(1) P_hip(1);Ps1(2) P_hip(2)];
V_linkw1 = [P_hip(1) Pw1(1);P_hip(2) Pw1(2)];
V_linkw2 = [Pw1(1) Psw_tip(1);Pw1(2) Psw_tip(2)];

% x CoM's of respective links
x_G = zeros(5);
x_G(1) = x1 + (l1/2)*cos(th1); 
x_G(2) = x1 + l1*cos(th1) + (l2/2)*cos(th1+th2);
x_G(3) = x1 + l1*cos(th1) + l2*cos(th1+th2) + (l3/2)*cos(th1 + th2 + th3);
x_G(4) = x1 + l1*cos(th1) + l2*cos(th1+th2) + l3*cos(th1 + th2 + th3) + (l4/2)*cos(th1 + th2 + th3 + th4);
x_G(5) = x1 + l1*cos(th1) + l2*cos(th1+th2) + (l5/2)*cos(th1 + th2 + th5);

% z_g CoM's of respective links
z_G = zeros(5);
z_G(1) = z1 + (l1/2)*sin(th1); 
z_G(2) = z1 + l1*sin(th1) + (l2/2)*sin(th1+th2);
z_G(3) = z1 + l1*sin(th1) + l2*sin(th1+th2) + (l3/2)*sin(th1 + th2 + th3);
z_G(4) = z1 + l1*sin(th1) + l2*sin(th1+th2) + l3*sin(th1 + th2 + th3) + (l4/2)*sin(th1 + th2 + th3 + th4);
z_G(5) = z1 + l1*sin(th1) + l2*sin(th1+th2) + (l5/2)*sin(th1 + th2 + th5);

xG = (m1*x_G(1) + m2*x_G(2) + m3*x_G(3) + m4*x_G(4) + m5*x_G(5))/(m1 + m2 + m3 + m4 + m5);
zG = (m1*z_G(1) + m2*z_G(2) + m3*z_G(3) + m4*z_G(4) + m5*z_G(5))/(m1 + m2 + m3 + m4 + m5);

plot(V_body(1,:),V_body(2,:));
xlim([-0.6 0.9])
ylim([0 1.5])
hold on
plot(V_links1(1,:),V_links1(2,:), 'k');
plot(V_links2(1,:),V_links2(2,:), 'k');
plot(V_linkw1(1,:),V_linkw1(2,:), 'k');
plot(V_linkw2(1,:),V_linkw2(2,:), 'k');

plot(xG, zG, 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'b');

plot(x_CoM_des, z_CoM_des, 'o', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
plot(x_sw_des, z_sw_des, 'o', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
grid on
hold off
end