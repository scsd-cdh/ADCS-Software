function B_body = magnetometer_model(B_eci, DCM_eci_to_body, noise_std, bias)
% MAGNETOMETER_MODEL Simulates a magnetometer sensor
%   B_body = MAGNETOMETER_MODEL(B_eci, DCM_eci_to_body, noise_std, bias)
%   
%   Inputs:
%       B_eci           - Magnetic field in ECI frame (3x1) [Tesla]
%       DCM_eci_to_body - DCM from ECI to body frame (3x3)
%       noise_std       - Standard deviation of measurement noise [Tesla]
%       bias            - Sensor bias (3x1) [Tesla]
%   
%   Output:
%       B_body - Measured magnetic field in body frame (3x1) [Tesla]

    % Transform magnetic field to body frame
    B_body_true = DCM_eci_to_body * B_eci;
    
    % Add noise and bias
    noise = noise_std * randn(3,1);
    B_body = B_body_true + bias + noise;
end
