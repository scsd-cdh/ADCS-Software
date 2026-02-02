function q = euler_to_quat(roll, pitch, yaw)
% EULER_TO_QUAT Convert Euler angles to quaternion
%   q = EULER_TO_QUAT(roll, pitch, yaw) converts Euler angles to quaternion
%   
%   Inputs:
%       roll  - Roll angle in radians
%       pitch - Pitch angle in radians  
%       yaw   - Yaw angle in radians
%   
%   Output:
%       q - Quaternion [q0; q1; q2; q3] (4x1)

    % Calculate half angles
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    
    % Compute quaternion components
    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
    
    q = [q0; q1; q2; q3];
    
    % Normalize
    q = q / norm(q);
end
