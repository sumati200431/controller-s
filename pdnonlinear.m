% PD CONTROLLER NON LINEAR 



clc; clear; close all;
g = 9.81;
ms = 0.05;
Ib = 0.01;
beam_length = 2;
r_desired = 0.5;
K1 = [14 0 10 0];     % position and angle
K2 = [0 3 0 2];       % velocity terms (r_dot, theta_dot)
tau_max = 4;
theta_max = deg2rad(45);
ball_friction = 0.8;
beam_friction = 0.5;
dt = 0.01; t_end = 20;
tspan = 0:dt:t_end;
x0 = [0; 0; 0.05; 0];
X = zeros(length(tspan), 4);
U = zeros(length(tspan), 1);
X(1,:) = x0';
for i = 1:length(tspan)-1
    x = X(i,:)';
    r = x(1); r_dot = x(2); theta = x(3); theta_dot = x(4);
    x_desired = [r_desired; 0; 0; 0];
    dx_desired = [0; 0; 0; 0];
    e = x - x_desired;
    dx = [0; r_dot; 0; theta_dot];
    tau = -K1 * e - K2 * dx;
    tau = max(min(tau, tau_max), -tau_max);
    dx_state = zeros(4,1);
    dx_state(1) = r_dot;
    dx_state(2) = (g*sin(theta) - r*theta_dot^2*cos(theta))*5/7 - ball_friction*r_dot;
    dx_state(3) = theta_dot;
    dx_state(4) = (tau - ms * g * r * cos(theta) - 2 * ms * r * r_dot * theta_dot - beam_friction * theta_dot) / (Ib + ms * r^2);
    X(i+1,:) = X(i,:) + dx_state'*dt;
    if abs(X(i+1,3)) > theta_max
        X(i+1,3) = sign(X(i+1,3)) * theta_max;
        X(i+1,4) = 0;
    end
    if abs(X(i+1,1)) > beam_length/2
        X(i+1,1) = sign(X(i+1,1)) * beam_length/2;
        X(i+1,2) = 0;
    end
    U(i) = tau;
end
ball_pos = X(:,1);
beam_angle = X(:,3);
figure;
plot(tspan, ball_pos, 'b', 'LineWidth', 2); hold on;
yline(r_desired, '--r', 'Target');
xlabel('Time (s)'); ylabel('Ball Position (m)');
title('Ball Position vs Time'); grid on;
figure;
plot(tspan, rad2deg(beam_angle), 'm', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Beam Angle (deg)');
title('Beam Angle vs Time'); grid on;
figure;
plot(tspan(1:end-1), U(1:end-1), 'k', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Control Torque (Nm)');
title('Torque vs Time'); grid on;
