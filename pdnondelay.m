clc; clear; close all;

% System parameters
g = 9.81;
ms = 0.05;
Ib = 0.01;
beam_length = 2;

% Controller gains
r_desired = 0.5;
K1 = [14 0 10 0];     % position and angle
K2 = [0 3 0 2];    % r_dot and theta_dot

       % Gains on dx/dt (i.e., velocities)
tau_max = 4;
theta_max = deg2rad(45);

% Damping
ball_friction = 0.8;
beam_friction = 0.5;

% Simulation settings
dt = 0.01; t_end = 20;
tspan = 0:dt:t_end;

% Initial state: [r; r_dot; theta; theta_dot]
x0 = [0; 0; 0.05; 0];
X = zeros(length(tspan), 4);
U = zeros(length(tspan), 1);
X(1,:) = x0';

control_delay = 0.02;                     % Delay in seconds(0.02 ,0.03 ,0.04)
delay_steps = round(control_delay / dt);
% Buffer to store past states
x_buffer = repmat(x0, 1, delay_steps + 1);  % initialize with x0


for i = 1:length(tspan)-1
    x = X(i,:)';
    r = x(1); r_dot = x(2); theta = x(3); theta_dot = x(4);

    % Desired state and velocity: [r; r_dot; theta; theta_dot]
    x_desired = [r_desired; 0; 0; 0];
    dx_desired = [0; 0; 0; 0];

    e = x - x_desired;
    dx = [0; r_dot; 0; theta_dot];  % Only derivative terms
 % Get delayed state(using the data present before the delay step to
    % compute control input
    if i > delay_steps % have enough of past data
        x_delayed = X(i - delay_steps, :)';
    else
        x_delayed = X(1,:)';  % still in the beginning of simulation and no past data
    end

  
    % Control law: u = -K1*x - K2*x_dot
     e_delayed = x_delayed - [r_desired; 0; 0; 0];
    tau = -K1 * e_delayed - K2 * dx;
    tau = max(min(tau, tau_max), -tau_max);

   % Nonlinear dynamics
    dx_state = zeros(4,1);
    dx_state(1) = r_dot;
    dx_state(2) = (g*sin(theta) - r*theta_dot^2*cos(theta))*5/7 - ball_friction*r_dot;
    dx_state(3) = theta_dot;
    dx_state(4) = (tau - ms * g * r * cos(theta) - 2 * ms * r * r_dot * theta_dot - beam_friction * theta_dot) / (Ib + ms * r^2);

    % Integration
    X(i+1,:) = X(i,:) + dx_state'*dt;

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

% Plotting results
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

% Animation
skip = 50;
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
    plot([r_desired r_desired], [-0.05 0.05], 'r--', 'LineWidth', 2); 

    xlim([-1.2, 1.2]*beam_length/2);
    ylim([-0.4, 0.4]*beam_length);
    title(sprintf('Time = %.2f s', tspan(idx)));
    xlabel('X (m)'); ylabel('Y (m)');
    
    drawnow;
    pause(0.01);
end


% === Final Output ===
final_ball_position = X(end, 1);
final_beam_angle_rad = X(end, 3);
final_beam_angle_deg = rad2deg(final_beam_angle_rad);

fprintf('Final ball position: %.4f m\n', final_ball_position);
fprintf('Final beam angle: %.4f rad (%.2f deg)\n', final_beam_angle_rad, final_beam_angle_deg);