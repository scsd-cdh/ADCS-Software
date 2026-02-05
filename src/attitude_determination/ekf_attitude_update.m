function [q_est, P_est] = ekf_attitude_update(q_prev, P_prev, omega, dt, z, h, R, Q)
% EKF_ATTITUDE_UPDATE Extended Kalman Filter for attitude estimation
%   [q_est, P_est] = EKF_ATTITUDE_UPDATE(q_prev, P_prev, omega, dt, z, h, R, Q)
%   
%   Inputs:
%       q_prev - Previous quaternion estimate (4x1)
%       P_prev - Previous error covariance (3x3)
%       omega  - Angular velocity measurement (3x1) [rad/s]
%       dt     - Time step [seconds]
%       z      - Measurement vector (nx1)
%       h      - Measurement model function handle
%       R      - Measurement noise covariance (nxn)
%       Q      - Process noise covariance (3x3)
%   
%   Outputs:
%       q_est - Updated quaternion estimate (4x1)
%       P_est - Updated error covariance (3x3)

    % --- Prediction Step ---
    % Propagate quaternion using angular velocity
    omega_norm = norm(omega);
    
    if omega_norm < 1e-8
        % Small angle approximation
        Omega = [0, -omega'; omega, -skew_symmetric(omega)];
        q_pred = expm(0.5 * Omega * dt) * q_prev;
    else
        % Exact propagation
        phi = omega_norm * dt;
        q_omega = [cos(phi/2); (omega/omega_norm)*sin(phi/2)];
        q_pred = quat_mult(q_omega, q_prev);
    end
    
    q_pred = q_pred / norm(q_pred);
    
    % Propagate covariance
    F = eye(3) - skew_symmetric(omega) * dt;
    P_pred = F * P_prev * F' + Q * dt;
    
    % --- Update Step ---
    % Compute expected measurement
    z_pred = h(q_pred);
    
    % Innovation
    y = z - z_pred;
    
    % Measurement Jacobian (simplified)
    H = compute_measurement_jacobian(q_pred, h);
    
    % Kalman gain
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    
    % Update error state
    delta_theta = K * y;
    
    % Update quaternion
    delta_q = [1; 0.5*delta_theta];
    delta_q = delta_q / norm(delta_q);
    q_est = quat_mult(delta_q, q_pred);
    q_est = q_est / norm(q_est);
    
    % Update covariance
    P_est = (eye(3) - K * H) * P_pred;
end

function S = skew_symmetric(v)
% SKEW_SYMMETRIC Create skew-symmetric matrix
    S = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end

function H = compute_measurement_jacobian(q, h)
% COMPUTE_MEASUREMENT_JACOBIAN Numerical Jacobian computation
    epsilon = 1e-6;
    z0 = h(q);
    n = length(z0);
    H = zeros(n, 3);
    
    for i = 1:3
        delta = zeros(3,1);
        delta(i) = epsilon;
        delta_q = [1; 0.5*delta];
        delta_q = delta_q / norm(delta_q);
        q_pert = quat_mult(delta_q, q);
        z_pert = h(q_pert);
        H(:,i) = (z_pert - z0) / epsilon;
    end
end
