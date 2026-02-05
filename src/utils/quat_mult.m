function q_out = quat_mult(q1, q2)
% QUAT_MULT Quaternion multiplication
%   q_out = QUAT_MULT(q1, q2) multiplies two quaternions q1 and q2
%   
%   Inputs:
%       q1 - First quaternion [q0; q1; q2; q3] (4x1)
%       q2 - Second quaternion [q0; q1; q2; q3] (4x1)
%   
%   Output:
%       q_out - Product quaternion (4x1)
%
%   Quaternion format: q = [q0; qv] where q0 is scalar, qv is vector part

    % Extract scalar and vector parts
    q1_s = q1(1);
    q1_v = q1(2:4);
    
    q2_s = q2(1);
    q2_v = q2(2:4);
    
    % Quaternion multiplication formula
    q_out_s = q1_s*q2_s - dot(q1_v, q2_v);
    q_out_v = q1_s*q2_v + q2_s*q1_v + cross(q1_v, q2_v);
    
    % Combine scalar and vector parts
    q_out = [q_out_s; q_out_v];
    
    % Normalize the output quaternion
    q_out = q_out / norm(q_out);
end
