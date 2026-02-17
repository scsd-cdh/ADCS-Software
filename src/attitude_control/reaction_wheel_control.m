function [omega_cmd, e_integral] = reaction_wheel_control(q_current, q_desired, omega, omega_desired, ...
                                                          e_integral, dt, Kp, Ki, Kd)
% REACTION_WHEEL_CONTROL Reaction wheel control for attitude control
%   [omega_cmd, e_integral] = REACTION_WHEEL_CONTROL(q_current, q_desired, omega, omega_desired, ...
%                                                     e_integral, dt, Kp, Ki, Kd)
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
%   
%   Outputs:
%       omega_cmd  - Commanded wheel angular velocity (3x1) [rad/s]
%       e_integral - Updated integral of error (3x1)

    % Compute quaternion error
    q_desired_conj = quat_conj(q_desired);
    q_error = quat_mult(q_current, q_desired_conj);
    
    % Extract vector part as attitude error
    e_attitude = q_error(2:4) * sign(q_error(1));
    
    % Angular velocity error
    e_omega = omega - omega_desired;
    
    % Update integral error
    e_integral = e_integral + e_attitude * dt;
    
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
    
    % PID control law for wheel velocity
    omega_cmd = -Kp * e_attitude - Ki * e_integral - Kd * e_omega;
    
    % Limit wheel speed (typical max speed for CubeSat wheels)
    max_wheel_speed = 5000 * (2*pi/60);  % 5000 RPM in rad/s
    for i = 1:3
        if abs(omega_cmd(i)) > max_wheel_speed
            omega_cmd(i) = sign(omega_cmd(i)) * max_wheel_speed;
        end
    end
end
