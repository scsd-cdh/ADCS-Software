function q = triad_algorithm(v1_body, v1_ref, v2_body, v2_ref)
% TRIAD_ALGORITHM TRIAD attitude determination algorithm
%   q = TRIAD_ALGORITHM(v1_body, v1_ref, v2_body, v2_ref)
%   
%   Computes attitude quaternion using two vector measurements
%   
%   Inputs:
%       v1_body - First vector in body frame (3x1, e.g., sun vector)
%       v1_ref  - First vector in reference frame (3x1)
%       v2_body - Second vector in body frame (3x1, e.g., mag field)
%       v2_ref  - Second vector in reference frame (3x1)
%   
%   Output:
%       q - Attitude quaternion from reference to body frame (4x1)

    % Normalize input vectors
    v1_body = v1_body / norm(v1_body);
    v1_ref = v1_ref / norm(v1_ref);
    v2_body = v2_body / norm(v2_body);
    v2_ref = v2_ref / norm(v2_ref);
    
    % Build triads in body frame
    r1_body = v1_body;
    r2_body = cross(v1_body, v2_body);
    r2_body = r2_body / norm(r2_body);
    r3_body = cross(r1_body, r2_body);
    
    M_body = [r1_body, r2_body, r3_body];
    
    % Build triads in reference frame
    r1_ref = v1_ref;
    r2_ref = cross(v1_ref, v2_ref);
    r2_ref = r2_ref / norm(r2_ref);
    r3_ref = cross(r1_ref, r2_ref);
    
    M_ref = [r1_ref, r2_ref, r3_ref];
    
    % Compute DCM from reference to body
    DCM = M_body * M_ref';
    
    % Convert DCM to quaternion
    q = dcm_to_quat(DCM);
end

function q = dcm_to_quat(DCM)
% DCM_TO_QUAT Convert Direction Cosine Matrix to quaternion
    trace_DCM = trace(DCM);
    
    if trace_DCM > 0
        s = 0.5 / sqrt(trace_DCM + 1.0);
        q0 = 0.25 / s;
        q1 = (DCM(3,2) - DCM(2,3)) * s;
        q2 = (DCM(1,3) - DCM(3,1)) * s;
        q3 = (DCM(2,1) - DCM(1,2)) * s;
    elseif (DCM(1,1) > DCM(2,2)) && (DCM(1,1) > DCM(3,3))
        s = 2.0 * sqrt(1.0 + DCM(1,1) - DCM(2,2) - DCM(3,3));
        q0 = (DCM(3,2) - DCM(2,3)) / s;
        q1 = 0.25 * s;
        q2 = (DCM(1,2) + DCM(2,1)) / s;
        q3 = (DCM(1,3) + DCM(3,1)) / s;
    elseif DCM(2,2) > DCM(3,3)
        s = 2.0 * sqrt(1.0 + DCM(2,2) - DCM(1,1) - DCM(3,3));
        q0 = (DCM(1,3) - DCM(3,1)) / s;
        q1 = (DCM(1,2) + DCM(2,1)) / s;
        q2 = 0.25 * s;
        q3 = (DCM(2,3) + DCM(3,2)) / s;
    else
        s = 2.0 * sqrt(1.0 + DCM(3,3) - DCM(1,1) - DCM(2,2));
        q0 = (DCM(2,1) - DCM(1,2)) / s;
        q1 = (DCM(1,3) + DCM(3,1)) / s;
        q2 = (DCM(2,3) + DCM(3,2)) / s;
        q3 = 0.25 * s;
    end
    
    q = [q0; q1; q2; q3];
    q = q / norm(q);
end
