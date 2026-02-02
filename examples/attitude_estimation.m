% Attitude Estimation using TRIAD Algorithm
% This script demonstrates attitude determination from sensor measurements

clear all;
close all;

% Initialize ADCS system
run('../init_adcs.m');

%% Simulation Setup
fprintf('\n========================================\n');
fprintf('Attitude Estimation using TRIAD\n');
fprintf('========================================\n\n');

% Time parameters
dt = config.simulation.dt;
t_end = 300;  % 5 minutes
time = 0:dt:t_end;
N = length(time);

%% Initialize State
% True attitude (unknown to estimator)
q_true = euler_to_quat(deg2rad(45), deg2rad(30), deg2rad(-20));
DCM_true = quat_to_dcm(q_true);

%% Reference Vectors (in ECI frame)
% Sun vector (fixed for simplicity)
sun_eci = [1; 0; 0];  % Sun direction

% Storage arrays
q_est_history = zeros(4, N);
q_true_history = zeros(4, N);
estimation_error = zeros(N, 1);

%% Simulation Loop
fprintf('Running attitude estimation...\n');

for i = 1:N
    % Update true attitude (simulate spacecraft rotation)
    omega_true = [0.01; 0.005; -0.008];  % Slow rotation
    omega_norm = norm(omega_true);
    if omega_norm > 1e-8
        phi = omega_norm * dt;
        q_omega = [cos(phi/2); (omega_true/omega_norm)*sin(phi/2)];
        q_true = quat_mult(q_omega, q_true);
        q_true = q_true / norm(q_true);
    end
    DCM_true = quat_to_dcm(q_true);
    
    % Magnetic field vector (time-varying in ECI)
    theta = 2*pi * time(i) / config.orbit.period;
    mag_eci = 2e-5 * [cos(theta); sin(theta); 0.5];
    mag_eci = mag_eci / norm(mag_eci);
    
    % Simulated sensor measurements (in body frame)
    sun_body = sun_sensor_model(sun_eci, DCM_true, ...
                                config.sensors.sun_sensor.noise_std, ...
                                config.sensors.sun_sensor.fov_half_angle);
    
    mag_body = magnetometer_model(mag_eci, DCM_true, ...
                                  config.sensors.magnetometer.noise_std, ...
                                  config.sensors.magnetometer.bias);
    
    % Check if sun sensor has valid measurement
    if any(isnan(sun_body))
        % Use previous estimate if sun not visible
        if i > 1
            q_est = q_est_history(:, i-1);
        else
            q_est = [1; 0; 0; 0];
        end
    else
        % TRIAD algorithm
        q_est = triad_algorithm(sun_body, sun_eci, mag_body, mag_eci);
    end
    
    % Compute estimation error
    q_error = quat_mult(q_est, quat_conj(q_true));
    estimation_error(i) = 2 * acos(min(abs(q_error(1)), 1)) * 180/pi;
    
    % Store data
    q_est_history(:, i) = q_est;
    q_true_history(:, i) = q_true;
end

fprintf('Simulation complete!\n\n');

%% Results
fprintf('Results:\n');
fprintf('  Mean estimation error:   %.3f degrees\n', mean(estimation_error));
fprintf('  Max estimation error:    %.3f degrees\n', max(estimation_error));
fprintf('  Std estimation error:    %.3f degrees\n', std(estimation_error));

%% Plotting
figure('Name', 'Attitude Estimation Results', 'Position', [100, 100, 1200, 600]);

% Estimation error
subplot(1,2,1);
plot(time, estimation_error);
grid on;
xlabel('Time [s]');
ylabel('Estimation Error [deg]');
title('TRIAD Attitude Estimation Error');

% True vs Estimated quaternion
subplot(1,2,2);
hold on;
plot(time, q_true_history', 'LineWidth', 1.5);
plot(time, q_est_history', '--', 'LineWidth', 1);
grid on;
xlabel('Time [s]');
ylabel('Quaternion Components');
title('True vs Estimated Attitude');
legend('q_0 true', 'q_1 true', 'q_2 true', 'q_3 true', ...
       'q_0 est', 'q_1 est', 'q_2 est', 'q_3 est', 'Location', 'best');

sgtitle('TRIAD Attitude Determination Algorithm');

fprintf('\nPlots generated. See Figure 1.\n');
