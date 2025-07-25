% NON LINEARIZED STATE FEEDBACK
clc; clear; close all;
g = 9.81;                   % gravity (m/s^2)
ms = 0.05;                  % ball mass (kg)
Ib = 0.01;                  % beam inertia
beam_length = 2;            % beam length (±1 m)
r_desired = 0.6;            % target position (m)
K = [8 5 3.5 2];   % Slower, more stable response
Ki = 1;           % Gentle integral action                 % integral gain
tau_max = 4;                % max torque (Nm)
theta_max = deg2rad(45);    % max beam angle ±45°
z_max = 0.5;                % anti-windup for integral
ball_friction = 0.8;        % damping for ball motion
beam_friction = 0.5;        % damping for beam rotation
dt = 0.01; t_end =50;
tspan = 0:dt:t_end;
x0 = [0; 0; 0.0; 0];
X = zeros(length(tspan), 4);
U = zeros(length(tspan), 1);
z = zeros(length(tspan), 1);
X(1,:) = x0';
for i = 1:length(tspan)-1
    x = X(i,:)';
    r = x(1); r_dot = x(2); theta = x(3); theta_dot = x(4);
    e_pos = r_desired - r;
    z(i+1) = z(i) + e_pos * dt;
    z(i+1) = min(max(z(i+1), -z_max), z_max);
    e = x - [r_desired; 0; 0; 0];
    tau = -K * e + Ki * z(i);
    tau = max(min(tau, tau_max), -tau_max);  
    dx = zeros(4,1);
    dx(1) = r_dot;
    dx(2) = (g*sin(theta) - r*theta_dot^2*cos(theta))*5/7 - ball_friction*r_dot;
    dx(3) = theta_dot;
    dx(4) = (tau - ms * g * r * cos(theta) - 2 * ms * r * r_dot * theta_dot - beam_friction * theta_dot) / (Ib + ms * r^2);
    X(i+1,:) = X(i,:) + dx'*dt;
    if abs(X(i+1,3)) > theta_max
        X(i+1,3) = sign(X(i+1,3)) * theta_max;
        X(i+1,4) = 0;  % stop rotation if limit hit
    end
    if abs(X(i+1,1)) > beam_length/2
        X(i+1,1) = sign(X(i+1,1)) * beam_length/2;
        X(i+1,2) = 0;
    end
    U(i) = tau;
end
ball_pos = X(:,1);
beam_angle = X(:,3);
figure(1);
plot(tspan, ball_pos, 'b', 'LineWidth', 2); hold on;
yline(r_desired, '--r', 'Target');
xlabel('Time (s)'); ylabel('Ball Position (m)');
title('Ball Position vs Time'); grid on;
figure(2);
plot(tspan, rad2deg(beam_angle), 'm', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Beam Angle (deg)');
title('Beam Angle vs Time'); grid on;
figure(3);
plot(tspan(1:end-1), U(1:end-1), 'k', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Control Torque (Nm)');
title('Torque vs Time'); grid on;
