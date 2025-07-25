clc; clear; close all;

% System parameters
g = 9.81;                
ms = 0.05;               
Ib = 0.01;              
beam_length = 2;        

% Controller
r_desired = 0.5;
K = [8 5 3.5 2];
Ki = 0.45;
tau_max = 4;
theta_max = deg2rad(45);
z_max = 0.5;

% Damping
ball_friction = 0.8;
beam_friction = 0.5;

% Simulation settings
dt = 0.01; t_end = 50;
tspan = 0:dt:t_end;

% Linearized system around r = r_desired, θ = 0
x0 = [0.25; 0; 0.05; 0];
X = zeros(length(tspan), 4);
U = zeros(length(tspan), 1);
z = zeros(length(tspan), 1);
X(1,:) = x0';

control_delay = 0.02;                     % Delay in seconds(0.02 ,0.03 ,0.04)
delay_steps = round(control_delay / dt); % Delay in discrete steps

% Buffer to store past states
x_buffer = repmat(x0, 1, delay_steps + 1);  % initialize with x0

for i = 1:length(tspan)-1
    x = X(i,:)';
    r = x(1); r_dot = x(2); theta = x(3); theta_dot = x(4);

    % Update integral of error
    e_pos = r_desired - r;
    z(i+1) = z(i) + e_pos * dt;
    z(i+1) = min(max(z(i+1), -z_max), z_max);

    % ------------------------------
    % Get delayed state
    if i > delay_steps
        x_delayed = X(i - delay_steps, :)';
    else
        x_delayed = X(1,:)';  % if not enough delay history yet
    end

    % Compute control using delayed state
    e_delayed = x_delayed - [r_desired; 0; 0; 0];
    tau = -K * e_delayed + Ki * z(i);
    tau = max(min(tau, tau_max), -tau_max);

    % Linearized dynamics
    dx = zeros(4,1);
    dx(1) = r_dot;
    dx(2) = (5/7) * g * theta - ball_friction * r_dot;  % linearized sin(θ) ≈ θ
    dx(3) = theta_dot;
    dx(4) = (tau - ms * g * r_desired * theta - beam_friction * theta_dot) / (Ib + ms * r_desired^2);

    % Integration
    X(i+1,:) = X(i,:) + dx'*dt;

    % Beam angle limits
    if abs(X(i+1,3)) > theta_max
        X(i+1,3) = sign(X(i+1,3)) * theta_max;
        X(i+1,4) = 0;
    end

    % Keep ball on beam
    if abs(X(i+1,1)) > beam_length/2
        X(i+1,1) = sign(X(i+1,1)) * beam_length/2;
        X(i+1,2) = 0;
    end

    U(i) = tau;
end

% Plot results
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

skip = 50;  % skip frames to speed up
figure;
for idx = 1:skip:length(tspan)
    theta_k = beam_angle(idx); 
    r_k = ball_pos(idx);

    % Beam coordinates
    xb = [-beam_length/2, beam_length/2];
    yb = [0, 0];
    R = [cos(theta_k) -sin(theta_k); sin(theta_k) cos(theta_k)];
    beam_coords = R * [xb; yb];

    % Ball position
    ball_xy = R * [r_k; 0];

    clf('reset'); hold on; axis equal;
    plot(beam_coords(1,:), beam_coords(2,:), 'k-', 'LineWidth', 4);
    plot(ball_xy(1), ball_xy(2), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
    plot([r_desired r_desired], [-0.05 0.05], 'r--', 'LineWidth', 2);  % visible target

    xlim([-1.2, 1.2]*beam_length/2);
    ylim([-0.4, 0.4]*beam_length);
    title(sprintf('Time = %.2f s', tspan(idx)));
    xlabel('X (m)'); ylabel('Y (m)');
    
    drawnow;
    pause(0.01);  % optional: smooth playback
end

% === Final Output ===
final_ball_position = X(end, 1);
final_beam_angle_rad = X(end, 3);
final_beam_angle_deg = rad2deg(final_beam_angle_rad);

fprintf('Final ball position: %.4f m\n', final_ball_position);
fprintf('Final beam angle: %.4f rad (%.2f deg)\n', final_beam_angle_rad, final_beam_angle_deg);