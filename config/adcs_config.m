function config = adcs_config()
% ADCS_CONFIG Configuration parameters for CubeSat ADCS
%   config = ADCS_CONFIG() returns a structure containing all ADCS parameters

    %% Spacecraft Physical Properties
    config.spacecraft.mass = 3.0;  % [kg] - 3U CubeSat
    
    % Moment of inertia matrix [kg·m^2]
    config.spacecraft.inertia = diag([0.02, 0.02, 0.01]);
    
    % Spacecraft dimensions [m]
    config.spacecraft.dimensions = [0.1, 0.1, 0.3];  % 1U x 1U x 3U
    
    %% Sensor Parameters
    % Magnetometer
    config.sensors.magnetometer.noise_std = 1e-8;  % [Tesla]
    config.sensors.magnetometer.bias = [1e-9; 1e-9; 1e-9];  % [Tesla]
    config.sensors.magnetometer.sample_rate = 10;  % [Hz]
    
    % Sun sensor
    config.sensors.sun_sensor.noise_std = 0.01;  % [radians]
    config.sensors.sun_sensor.fov_half_angle = deg2rad(60);  % [radians]
    config.sensors.sun_sensor.sample_rate = 5;  % [Hz]
    
    % Gyroscope
    config.sensors.gyroscope.noise_std = 1e-4;  % [rad/s]
    config.sensors.gyroscope.bias = [1e-5; 1e-5; 1e-5];  % [rad/s]
    config.sensors.gyroscope.sample_rate = 100;  % [Hz]
    
    %% Actuator Parameters
    % Magnetorquers
    config.actuators.magnetorquer.max_dipole = 0.2;  % [A·m^2]
    config.actuators.magnetorquer.noise_std = 1e-3;  % [A·m^2]
    config.actuators.magnetorquer.number = 3;  % 3-axis control
    
    % Reaction wheels
    config.actuators.reaction_wheel.max_torque = 1e-3;  % [N·m]
    config.actuators.reaction_wheel.max_speed = 5000 * (2*pi/60);  % [rad/s]
    config.actuators.reaction_wheel.inertia = 1e-5;  % [kg·m^2]
    config.actuators.reaction_wheel.friction = [1e-7; 1e-7; 1e-7];  % [N·m]
    config.actuators.reaction_wheel.noise_std = 1e-6;  % [N·m]
    
    %% Control Parameters
    % B-dot controller
    config.control.bdot.gain = 1e5;
    
    % PID controller gains
    config.control.pid.Kp = 0.1;
    config.control.pid.Ki = 0.01;
    config.control.pid.Kd = 0.5;
    
    % Reaction wheel controller gains
    config.control.reaction_wheel.Kp = 2.0;
    config.control.reaction_wheel.Ki = 0.1;
    config.control.reaction_wheel.Kd = 1.0;
    
    %% Attitude Determination Parameters
    % EKF parameters
    config.estimation.ekf.process_noise = diag([1e-8, 1e-8, 1e-8]);
    config.estimation.ekf.measurement_noise = diag([1e-6, 1e-6, 1e-6]);
    config.estimation.ekf.initial_covariance = diag([0.1, 0.1, 0.1]);
    
    %% Orbital Parameters
    config.orbit.altitude = 400e3;  % [m] - 400 km
    config.orbit.inclination = deg2rad(51.6);  % [radians]
    config.orbit.period = 92.7 * 60;  % [seconds]
    
    %% Simulation Parameters
    config.simulation.dt = 0.1;  % [seconds]
    config.simulation.duration = 5400;  % [seconds] - 1.5 orbits
    config.simulation.initial_angular_velocity = [0.1; 0.05; -0.08];  % [rad/s]
    config.simulation.initial_attitude = [1; 0; 0; 0];  % quaternion
    
    %% Environmental Parameters
    % Earth magnetic field model (simplified dipole)
    config.environment.earth_radius = 6371e3;  % [m]
    config.environment.earth_magnetic_moment = 7.96e15;  % [A·m^2]
    
    % Solar constant
    config.environment.solar_constant = 1361;  % [W/m^2]
end
