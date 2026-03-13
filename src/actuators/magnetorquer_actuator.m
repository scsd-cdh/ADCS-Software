function tau = magnetorquer_actuator(M_cmd, B_body, noise_std)
% MAGNETORQUER_ACTUATOR Simulates magnetorquer actuator
%   tau = MAGNETORQUER_ACTUATOR(M_cmd, B_body, noise_std)
%   
%   Inputs:
%       M_cmd     - Commanded magnetic dipole moment (3x1) [A·m^2]
%       B_body    - Magnetic field in body frame (3x1) [Tesla]
%       noise_std - Standard deviation of actuator noise [A·m^2]
%   
%   Output:
%       tau - Generated torque (3x1) [N·m]

    % Add actuator noise
    M_actual = M_cmd + noise_std * randn(3,1);
    
    % Compute torque: tau = M x B
    tau = cross(M_actual, B_body);
end
