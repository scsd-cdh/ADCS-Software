function q_conj = quat_conj(q)
% QUAT_CONJ Quaternion conjugate
%   q_conj = QUAT_CONJ(q) computes the conjugate of quaternion q
%   
%   Input:
%       q - Quaternion [q0; q1; q2; q3] (4x1)
%   
%   Output:
%       q_conj - Conjugate quaternion [q0; -q1; -q2; -q3] (4x1)

    q_conj = [q(1); -q(2:4)];
end
