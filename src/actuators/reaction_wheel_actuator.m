function tau = reaction_wheel_actuator(omega_wheel, omega_wheel_cmd, dt, J_wheel, tau_friction, noise_std)
% REACTION_WHEEL_ACTUATOR Simulates reaction wheel actuator
%   tau = REACTION_WHEEL_ACTUATOR(omega_wheel, omega_wheel_cmd, dt, J_wheel, tau_friction, noise_std)
%   
%   Inputs:
%       omega_wheel     - Current wheel angular velocity (3x1) [rad/s]
%       omega_wheel_cmd - Commanded wheel angular velocity (3x1) [rad/s]
%       dt              - Time step [seconds]
%       J_wheel         - Wheel moment of inertia [kg·m^2]
%       tau_friction    - Friction torque (3x1) [N·m]
%       noise_std       - Standard deviation of torque noise [N·m]
%   
%   Output:
%       tau - Reaction torque on spacecraft body (3x1) [N·m]

    % Simple first-order wheel dynamics
    tau_motor = zeros(3,1);
    for i = 1:3
        % Motor torque to achieve commanded speed
        omega_error = omega_wheel_cmd(i) - omega_wheel(i);
        Kp_motor = 0.1;  % Motor proportional gain
        tau_motor(i) = Kp_motor * omega_error;
        
        % Apply friction
        if abs(omega_wheel(i)) > 1e-6
            tau_motor(i) = tau_motor(i) - sign(omega_wheel(i)) * tau_friction(i);
        end
    end
    
    % Reaction torque on spacecraft (Newton's third law)
    tau = -tau_motor + noise_std * randn(3,1);
    
    % Limit torque
    max_torque = 1e-3;  % [N·m]
    for i = 1:3
        if abs(tau(i)) > max_torque
            tau(i) = sign(tau(i)) * max_torque;
        end
    end
end
