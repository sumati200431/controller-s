% PID LINEARISED

clc; clear; close all;
g = 9.81;
ms = 0.05;
Ib = 0.01;
beam_length = 2;
% PID Controller gains
Kp_set = [18 5 6 7] ;
Ki_set = [8 7 8 9];
Kd_set = [10 11 12 13];
r_desired = 0.5;
tau_max = 4;
theta_max = deg2rad(45);
z_max = 0.5;
ball_friction = 0.8;
beam_friction = 0.5;
dt = 0.01; t_end = 50;
tspan = 0:dt:t_end;
x0 = [0; 0; 0; 0];
X = zeros(length(tspan), 4);
U = zeros(length(tspan), 1);
z = zeros(length(tspan), 1);  % integral error
X(1,:) = x0';
for i = 1:length(tspan)-1
    x = X(i,:)';
    r = x(1); r_dot = x(2); theta = x(3); theta_dot = x(4);
    if(i<5)
    Kp=Kp_set(i);
    Ki=Ki_set(i);
    Kd=Kd_set(i);
    end
    e_pos = r_desired - r;
    z(i+1) = z(i) + e_pos * dt;
    z(i+1) = min(max(z(i+1), -z_max), z_max);  % anti-windup
    theta_ref = Kp * e_pos + Ki * z(i) - Kd * r_dot;
    theta_ref = max(min(theta_ref, theta_max), -theta_max);  % saturate ref
   beam_Kp = 15;  % PD gains for beam angle control
   beam_Kd = 3;
   beam_error = theta_ref - theta;
   beam_error_dot = -theta_dot;
tau = beam_Kp * beam_error + beam_Kd * beam_error_dot + ms * g * r * cos(theta); %control input

    tau = max(min(tau, tau_max), -tau_max);
    dx = zeros(4,1);
    dx(1) = r_dot;
    dx(2) = (5/7)*g*theta - ball_friction * r_dot;
    dx(3) = theta_dot;
    dx(4) = (tau - ms * g * r_desired * theta - beam_friction * theta_dot) / (Ib + ms * r_desired^2);
    X(i+1,:) = X(i,:) + dx' * dt;
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
e_rad, final_beam_angle_deg); 