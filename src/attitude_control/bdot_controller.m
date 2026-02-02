function M_cmd = bdot_controller(B_body, B_body_prev, dt, k_bdot)
% BDOT_CONTROLLER B-dot detumbling control algorithm
%   M_cmd = BDOT_CONTROLLER(B_body, B_body_prev, dt, k_bdot)
%   
%   Implements B-dot control law for initial detumbling
%   
%   Inputs:
%       B_body      - Current magnetic field in body frame (3x1) [Tesla]
%       B_body_prev - Previous magnetic field in body frame (3x1) [Tesla]
%       dt          - Time step [seconds]
%       k_bdot      - B-dot control gain (scalar)
%   
%   Output:
%       M_cmd - Commanded magnetic dipole moment (3x1) [A·m^2]

    % Compute time derivative of magnetic field
    B_dot = (B_body - B_body_prev) / dt;
    
    % B-dot control law
    M_cmd = -k_bdot * B_dot;
    
    % Limit magnetic dipole moment (typical range for CubeSats)
    max_dipole = 0.2;  % [A·m^2]
    if norm(M_cmd) > max_dipole
        M_cmd = M_cmd / norm(M_cmd) * max_dipole;
    end
end
