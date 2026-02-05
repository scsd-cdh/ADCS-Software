% Detumbling Simulation using B-dot Control
% This script simulates the detumbling phase of a CubeSat using magnetorquers

clear all;
close all;

% Initialize ADCS system
run('../init_adcs.m');

%% Simulation Setup
fprintf('\n========================================\n');
fprintf('CubeSat Detumbling Simulation\n');
fprintf('========================================\n\n');

% Time parameters
dt = config.simulation.dt;
t_end = 3600;  % 1 hour simulation
time = 0:dt:t_end;
N = length(time);

%% Initialize State
% Initial attitude (random)
q = euler_to_quat(0.5, 0.3, -0.2);  % Initial attitude

% Initial angular velocity (high tumbling rate)
omega = [0.2; 0.15; -0.18];  % [rad/s]

% Previous magnetic field measurement
B_body_prev = [0; 0; 2e-5];  % Initial guess

%% Storage Arrays
q_history = zeros(4, N);
omega_history = zeros(3, N);
M_cmd_history = zeros(3, N);
tau_history = zeros(3, N);

%% Simulation Loop
fprintf('Running detumbling simulation...\n');

for i = 1:N
    % Simple Earth magnetic field model (dipole in ECI frame)
    theta = 2*pi * time(i) / config.orbit.period;
    B_eci = 2e-5 * [cos(theta); sin(theta); 0.5];
    
    % Convert to body frame
    DCM = quat_to_dcm(q);
    B_body = DCM * B_eci;
    
    % B-dot controller
    M_cmd = bdot_controller(B_body, B_body_prev, dt, config.control.bdot.gain);
    
    % Magnetorquer actuator
    tau = magnetorquer_actuator(M_cmd, B_body, config.actuators.magnetorquer.noise_std);
    
    % Store data
    q_history(:, i) = q;
    omega_history(:, i) = omega;
    M_cmd_history(:, i) = M_cmd;
    tau_history(:, i) = tau;
    
    % Propagate dynamics (simplified)
    omega_dot = config.spacecraft.inertia \ (tau - cross(omega, config.spacecraft.inertia * omega));
    omega = omega + omega_dot * dt;
    
    % Propagate attitude
    omega_norm = norm(omega);
    if omega_norm > 1e-8
        phi = omega_norm * dt;
        q_omega = [cos(phi/2); (omega/omega_norm)*sin(phi/2)];
        q = quat_mult(q_omega, q);
        q = q / norm(q);
    end
    
    % Update previous measurement
    B_body_prev = B_body;
end

fprintf('Simulation complete!\n\n');

%% Results
fprintf('Results:\n');
fprintf('  Initial angular velocity: [%.3f, %.3f, %.3f] rad/s\n', omega_history(:,1));
fprintf('  Final angular velocity:   [%.3f, %.3f, %.3f] rad/s\n', omega_history(:,end));
fprintf('  Initial rate magnitude:   %.3f rad/s\n', norm(omega_history(:,1)));
fprintf('  Final rate magnitude:     %.3f rad/s\n', norm(omega_history(:,end)));
fprintf('  Reduction:                %.1f%%\n', ...
    (1 - norm(omega_history(:,end))/norm(omega_history(:,1)))*100);

%% Plotting
figure('Name', 'Detumbling Simulation Results', 'Position', [100, 100, 1200, 800]);

% Angular velocity
subplot(2,2,1);
plot(time/60, omega_history');
grid on;
xlabel('Time [min]');
ylabel('Angular Velocity [rad/s]');
title('Angular Velocity Components');
legend('\omega_x', '\omega_y', '\omega_z');

% Angular velocity magnitude
subplot(2,2,2);
plot(time/60, vecnorm(omega_history));
grid on;
xlabel('Time [min]');
ylabel('||\omega|| [rad/s]');
title('Angular Velocity Magnitude');

% Commanded magnetic dipole
subplot(2,2,3);
plot(time/60, M_cmd_history');
grid on;
xlabel('Time [min]');
ylabel('Magnetic Dipole [A·m^2]');
title('Commanded Magnetic Dipole Moment');
legend('M_x', 'M_y', 'M_z');

% Control torque
subplot(2,2,4);
plot(time/60, tau_history'*1e6);
grid on;
xlabel('Time [min]');
ylabel('Torque [μN·m]');
title('Control Torque');
legend('\tau_x', '\tau_y', '\tau_z');

sgtitle('B-dot Detumbling Controller Performance');

fprintf('\nPlots generated. See Figure 1.\n');
