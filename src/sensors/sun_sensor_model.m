function S_body = sun_sensor_model(S_eci, DCM_eci_to_body, noise_std, fov_half_angle)
% SUN_SENSOR_MODEL Simulates a sun sensor
%   S_body = SUN_SENSOR_MODEL(S_eci, DCM_eci_to_body, noise_std, fov_half_angle)
%   
%   Inputs:
%       S_eci           - Sun vector in ECI frame (3x1, unit vector)
%       DCM_eci_to_body - DCM from ECI to body frame (3x3)
%       noise_std       - Standard deviation of measurement noise [radians]
%       fov_half_angle  - Half-angle of field of view [radians]
%   
%   Output:
%       S_body - Measured sun vector in body frame (3x1, unit vector)
%                Returns NaN if sun is outside FOV

    % Transform sun vector to body frame
    S_body_true = DCM_eci_to_body * S_eci;
    S_body_true = S_body_true / norm(S_body_true);
    
    % Check if sun is within field of view (assume sensor along z-axis)
    angle_from_boresight = acos(S_body_true(3));
    
    if angle_from_boresight > fov_half_angle
        % Sun is outside FOV
        S_body = [NaN; NaN; NaN];
        return;
    end
    
    % Add angular noise
    noise_vec = noise_std * randn(3,1);
    S_body = S_body_true + noise_vec;
    S_body = S_body / norm(S_body);  % Renormalize
end
