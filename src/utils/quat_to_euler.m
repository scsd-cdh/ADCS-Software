function [roll, pitch, yaw] = quat_to_euler(q)
% QUAT_TO_EULER Convert quaternion to Euler angles
%   [roll, pitch, yaw] = QUAT_TO_EULER(q) converts a quaternion to Euler angles
%   
%   Input:
%       q - Quaternion [q0; q1; q2; q3] (4x1)
%   
%   Outputs:
%       roll  - Roll angle in radians
%       pitch - Pitch angle in radians
%       yaw   - Yaw angle in radians

    % Normalize quaternion
    q = q / norm(q);
    
    % Extract components
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
    
    % Calculate Euler angles
    roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    pitch = asin(2*(q0*q2 - q3*q1));
    yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
end
