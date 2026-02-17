function M_cmd = magnetorquer_control(tau_desired, B_body)
% MAGNETORQUER_CONTROL Compute magnetorquer commands from desired torque
%   M_cmd = MAGNETORQUER_CONTROL(tau_desired, B_body)
%   
%   Computes magnetic dipole moment to generate desired torque
%   
%   Inputs:
%       tau_desired - Desired control torque (3x1) [N·m]
%       B_body      - Magnetic field in body frame (3x1) [Tesla]
%   
%   Output:
%       M_cmd - Commanded magnetic dipole moment (3x1) [A·m^2]
%
%   Note: Torque from magnetorquers: tau = M x B
%         Only components perpendicular to B can be controlled

    % Compute pseudo-inverse solution
    B_norm = norm(B_body);
    
    if B_norm < 1e-9
        % Magnetic field too weak
        M_cmd = [0; 0; 0];
        return;
    end
    
    % Skew-symmetric matrix for cross product
    B_cross = [0, -B_body(3), B_body(2);
               B_body(3), 0, -B_body(1);
               -B_body(2), B_body(1), 0];
    
    % Pseudo-inverse solution (least-squares)
    M_cmd = pinv(B_cross) * tau_desired;
    
    % Limit magnetic dipole moment
    max_dipole = 0.2;  % [A·m^2]
    if norm(M_cmd) > max_dipole
        M_cmd = M_cmd / norm(M_cmd) * max_dipole;
    end
end
