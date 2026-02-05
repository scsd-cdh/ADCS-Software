function DCM = quat_to_dcm(q)
% QUAT_TO_DCM Convert quaternion to Direction Cosine Matrix
%   DCM = QUAT_TO_DCM(q) converts a quaternion to a DCM
%   
%   Input:
%       q - Quaternion [q0; q1; q2; q3] (4x1)
%   
%   Output:
%       DCM - Direction Cosine Matrix (3x3)

    % Normalize quaternion
    q = q / norm(q);
    
    % Extract components
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
    
    % Compute DCM elements
    DCM = zeros(3,3);
    
    DCM(1,1) = q0^2 + q1^2 - q2^2 - q3^2;
    DCM(1,2) = 2*(q1*q2 + q0*q3);
    DCM(1,3) = 2*(q1*q3 - q0*q2);
    
    DCM(2,1) = 2*(q1*q2 - q0*q3);
    DCM(2,2) = q0^2 - q1^2 + q2^2 - q3^2;
    DCM(2,3) = 2*(q2*q3 + q0*q1);
    
    DCM(3,1) = 2*(q1*q3 + q0*q2);
    DCM(3,2) = 2*(q2*q3 - q0*q1);
    DCM(3,3) = q0^2 - q1^2 - q2^2 + q3^2;
end
