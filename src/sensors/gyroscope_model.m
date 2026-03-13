function omega_meas = gyroscope_model(omega_true, noise_std, bias, dt)
% GYROSCOPE_MODEL Simulates a gyroscope sensor
%   omega_meas = GYROSCOPE_MODEL(omega_true, noise_std, bias, dt)
%   
%   Inputs:
%       omega_true - True angular velocity in body frame (3x1) [rad/s]
%       noise_std  - Standard deviation of measurement noise [rad/s]
%       bias       - Sensor bias (3x1) [rad/s]
%       dt         - Time step [seconds]
%   
%   Output:
%       omega_meas - Measured angular velocity (3x1) [rad/s]

    % Add white noise and bias
    noise = noise_std * randn(3,1);
    omega_meas = omega_true + bias + noise;
end
