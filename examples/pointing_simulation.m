% Attitude Pointing Simulation using PID Control
% This script simulates pointing a CubeSat to a desired attitude

clear all;
close all;

% Initialize ADCS system
run('../init_adcs.m');

%% Simulation Setup
fprintf('\n========================================\n');
fprintf('CubeSat Attitude Pointing Simulation\n');
fprintf('========================================\n\n');

% Time parameters
dt = config.simulation.dt;
t_end = 600;  % 10 minutes
time = 0:dt:t_end;
N = length(time);

%% Initialize State
% Initial attitude (off from desired)
q_current = euler_to_quat(deg2rad(30), deg2rad(20), deg2rad(-15));

% Desired attitude (nadir pointing)
q_desired = [1; 0; 0; 0];

% Initial angular velocity (small residual)
omega = [0.01; -0.01; 0.005];  % [rad/s]
omega_desired = [0; 0; 0];  % [rad/s]

% Controller state
e_integral = [0; 0; 0];

%% Storage Arrays
q_history = zeros(4, N);
omega_history = zeros(3, N);
tau_history = zeros(3, N);
attitude_error = zeros(N, 1);

%% Simulation Loop
fprintf('Running pointing simulation...\n');

for i = 1:N
    % PID attitude controller
    [tau_cmd, e_integral] = pid_attitude_controller(q_current, q_desired, omega, omega_desired, ...
                                                     e_integral, dt, ...
                                                     config.control.pid.Kp, ...
                                                     config.control.pid.Ki, ...
                                                     config.control.pid.Kd, ...
                                                     config.spacecraft.inertia);
    
    % For this simulation, assume tau_cmd is directly applied
    % (could use reaction wheels or magnetorquers in practice)
    tau = tau_cmd;
    
    % Compute attitude error
    q_error = quat_mult(q_current, quat_conj(q_desired));
    attitude_error(i) = 2 * acos(min(abs(q_error(1)), 1)) * 180/pi;  % degrees
    
    % Store data
    q_history(:, i) = q_current;
    omega_history(:, i) = omega;
    tau_history(:, i) = tau;
    
    % Propagate dynamics
    omega_dot = config.spacecraft.inertia \ (tau - cross(omega, config.spacecraft.inertia * omega));
    omega = omega + omega_dot * dt;
    
    % Propagate attitude
    omega_norm = norm(omega);
    if omega_norm > 1e-8
        phi = omega_norm * dt;
        q_omega = [cos(phi/2); (omega/omega_norm)*sin(phi/2)];
        q_current = quat_mult(q_omega, q_current);
        q_current = q_current / norm(q_current);
    end
end

fprintf('Simulation complete!\n\n');

%% Results
fprintf('Results:\n');
fprintf('  Initial attitude error: %.2f degrees\n', attitude_error(1));
fprintf('  Final attitude error:   %.4f degrees\n', attitude_error(end));
fprintf('  Settling time (< 1°):   %.1f seconds\n', ...
    time(find(attitude_error < 1, 1, 'first')));

%% Plotting
figure('Name', 'Pointing Simulation Results', 'Position', [100, 100, 1200, 800]);

% Attitude error
subplot(2,2,1);
plot(time, attitude_error);
grid on;
xlabel('Time [s]');
ylabel('Attitude Error [deg]');
title('Attitude Error');
yline(1, '--r', 'Target: 1°');

% Quaternion components
subplot(2,2,2);
plot(time, q_history');
grid on;
xlabel('Time [s]');
ylabel('Quaternion Components');
title('Attitude Quaternion');
legend('q_0', 'q_1', 'q_2', 'q_3');

% Angular velocity
subplot(2,2,3);
plot(time, omega_history');
grid on;
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
title('Angular Velocity');
legend('\omega_x', '\omega_y', '\omega_z');

% Control torque
subplot(2,2,4);
plot(time, tau_history'*1e6);
grid on;
xlabel('Time [s]');
ylabel('Torque [μN·m]');
title('Control Torque');
legend('\tau_x', '\tau_y', '\tau_z');

sgtitle('PID Attitude Pointing Controller Performance');

fprintf('\nPlots generated. See Figure 1.\n');
