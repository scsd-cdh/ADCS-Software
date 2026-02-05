function [tau_cmd, e_integral] = pid_attitude_controller(q_current, q_desired, omega, omega_desired, ...
                                                         e_integral, dt, Kp, Ki, Kd, I_body)
% PID_ATTITUDE_CONTROLLER PID controller for attitude control
%   [tau_cmd, e_integral] = PID_ATTITUDE_CONTROLLER(q_current, q_desired, omega, omega_desired, ...
%                                                    e_integral, dt, Kp, Ki, Kd, I_body)
%   
%   Inputs:
%       q_current    - Current attitude quaternion (4x1)
%       q_desired    - Desired attitude quaternion (4x1)
%       omega        - Current angular velocity (3x1) [rad/s]
%       omega_desired- Desired angular velocity (3x1) [rad/s]
%       e_integral   - Integral of error (3x1)
%       dt           - Time step [seconds]
%       Kp           - Proportional gain (3x3 or scalar)
%       Ki           - Integral gain (3x3 or scalar)
%       Kd           - Derivative gain (3x3 or scalar)
%       I_body       - Moment of inertia matrix (3x3) [kg·m^2]
%   
%   Outputs:
%       tau_cmd    - Commanded control torque (3x1) [N·m]
%       e_integral - Updated integral of error (3x1)

    % Compute quaternion error
    q_desired_conj = quat_conj(q_desired);
    q_error = quat_mult(q_current, q_desired_conj);
    
    % Extract vector part as attitude error
    e_attitude = q_error(2:4) * sign(q_error(1));
    
    % Angular velocity error
    e_omega = omega - omega_desired;
    
    % Update integral error with anti-windup
    e_integral = e_integral + e_attitude * dt;
    
    % Limit integral term
    max_integral = 0.1;
    e_integral = min(max(e_integral, -max_integral), max_integral);
    
    % Ensure gains are matrices
    if isscalar(Kp)
        Kp = Kp * eye(3);
    end
    if isscalar(Ki)
        Ki = Ki * eye(3);
    end
    if isscalar(Kd)
        Kd = Kd * eye(3);
    end
    
    % PID control law
    tau_cmd = -Kp * e_attitude - Ki * e_integral - Kd * e_omega;
    
    % Add gyroscopic compensation
    tau_cmd = tau_cmd - cross(omega, I_body * omega);
    
    % Limit control torque (typical for CubeSat actuators)
    max_torque = 1e-3;  % [N·m]
    if norm(tau_cmd) > max_torque
        tau_cmd = tau_cmd / norm(tau_cmd) * max_torque;
    end
end
