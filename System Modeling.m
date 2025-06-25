% Parameters
mp = 0.285; mw = 0.025; Jp = 0.0010; Jw = 0.0013; rw = 0.022; 1 = 0.1;
= 0.0001; kt = 0.3; kb = 0.5; R = 5; g = 9.81; C
% System Matrices
M = [(Jw/rw^2)+mw+mp, mp*1; -mp, -(Jp/1)-mp*1];
K = [0, 0; 0, mp*g];
C = [(c/rw^2)+(kt*kb)/(R*rw^2), -(c/rw)-(kt*kb)/(R*rw); (c+1/R*kt*kb)/(rw*1), -(kt*kb)/(R*1)-(c/1)];
F = [kt/(R*rw); kt/(R*1)];
% State-Space Representation
A = [zeros(2), eye(2); -M\K, -M\C];
B = [zeros(2, 1); M\F];
C1 = eye(4);
D = zeros(4, 1);
x1 = eig(A);
x2 = B;
% Initial Conditions
q0 = [0; pi/60; 0; 0];
q10 = [0;10*pi/180;0;0];
% Simulation
SYS = ss (A, B, C1, D);
[y, t, q] = initial(SYS, q);
figure;
plot(t, q);
title('System Response to Initial Conditions');
xlabel('Time (s)');
ylabel('States');
legend('x[m]', 'Theta[rads]', 'Velocity [m/s]', 'Angular Velocity [rad/s]')
SYS = ss (A, B, C1, D);
[y, t, q] = initial(SYS, q0,0.1);
figure;
plot(t, q);
title('System Response to Initial Conditions');
xlabel('Time (s)');
ylabel('States');
legend('x[m]', 'Theta[rads]', 'Velocity [m/s]', 'Angular Velocity [rad/s]')
% Closed-Loop Controller Design
desired_poles = [-1, -2, -3, -4];
K = acker(A, B, desired_poles);
A_c1 = A -B * K;
SYS_c1 = ss(A_cl, B, C1, D);
v = -K*q';
[y_c1, t_cl, q_cl] = initial(SYS_cl, q0);
figure;
plot(t_cl, q_cl);
title('Closed-Loop System Response');
legend('x[m]', 'Theta[rads]', 'Velocity [m/s]', 'Angular Velocity [rad/s]');
xlabel('Time (s)');
ylabel('States');
[y_cl, t_cl, q_cl] = initial(SYS_cl, q0);
figure;
plot(t_cl, q_cl);
title('Closed-Loop System Response');
legend('x[m]', 'Theta[rads]', 'Velocity [m/s]', 'Angular Velocity [rad/s]');
xlabel('Time (s)');
ylabel('States');
[y_cl, t_cl, q_cl] = initial(SYS_c1, q10);
figure;
plot(t,v);
title('Closed-Loop System (Input Voltage Analysis)');
xlabel('Time (s)'):
ylabel('Input Voltage (v)');
% Closed-Loop System with Saturation
v_max = 5;
initial_conditions = linspace(0, 10, 5);
figure;
for i = 1: length(initial_conditions)
end
q0 = [deg2rad(initial_conditions(i)); 0; 0; 0];
[y, t, q] = initial(SYS_cl, q0);
subplot(2, 1, 1);
plot(t, rad2deg(q(:, 1)));
hold on;
subplot(2, 1, 2);
v-K q';
plot(t, v);
hold on;
subplot(2, 1, 1);
title('Closed-Loop System Response with Saturation');
xlabel('Time (s)')
ylabel('Angle q1 (degrees)');
legend('IC=0 deg', 'IC=2.5 deg', 'IC=5 deg', 'IC=7.5 deg', 'IC= 10 deg');
subplot(2, 1, 2);
xlabel('Time (s)');
ylabel('Control Input v');
title('Control Input for Closed-Loop System');
legend('IC=0 deg', 'IC=2.5 deg', 'IC=5 deg', 'IC=7.5 deg', 'IC= 10 deg');
% Linear Quadratic Regulator (LQR) Design
Q = diag([1, 100, 1, 100]):
R = 1;
K_lqr = lqr(A, B, Q, R);
A_closed_loop_lqr = A-B* K_1qr;
SYS_closed_loop_lqr = ss(A_closed_loop_lqr, B, C1, D);
EV_LQR = eig(A_closed_loop_lqr);
[y_1qr, t_lqr, q_lqr] = initial(SYS_closed_loop_1qr, q0);
figure; plot(t,q);xlabel('Time [s]'); ylabel('States');
title('Closed Loop System, LQR')
legend('x[m]', 'Theta[rads]', 'Velocity [m/s]', 'Angular Velocity [rad/s]')
legend('x[m]', 'Theta[rads]', 'Velocity [m/s]', 'Angular Velocity [rad/s]')
u = -K_LQR*q';
figure; plot(t,u)
xlabel('Time [s]'); ylabel('Input Voltage (V)'); title('Closed-loop System, LQR
