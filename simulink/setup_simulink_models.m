% ADCS Simulink Model Setup Script
% This script sets up the workspace for the ADCS Simulink models

% Load configuration
config = adcs_config();

% Define model parameters for Simulink
model_params.dt = config.simulation.dt;
model_params.t_end = config.simulation.duration;

% Spacecraft parameters
model_params.I_body = config.spacecraft.inertia;
model_params.mass = config.spacecraft.mass;

% Initial conditions
model_params.q_init = config.simulation.initial_attitude;
model_params.omega_init = config.simulation.initial_angular_velocity;

% Sensor parameters
model_params.mag_noise = config.sensors.magnetometer.noise_std;
model_params.mag_bias = config.sensors.magnetometer.bias;
model_params.gyro_noise = config.sensors.gyroscope.noise_std;
model_params.gyro_bias = config.sensors.gyroscope.bias;
model_params.sun_noise = config.sensors.sun_sensor.noise_std;
model_params.sun_fov = config.sensors.sun_sensor.fov_half_angle;

% Actuator parameters
model_params.mag_max = config.actuators.magnetorquer.max_dipole;
model_params.rw_max_torque = config.actuators.reaction_wheel.max_torque;
model_params.rw_max_speed = config.actuators.reaction_wheel.max_speed;
model_params.rw_inertia = config.actuators.reaction_wheel.inertia;

% Control parameters
model_params.bdot_gain = config.control.bdot.gain;
model_params.pid_Kp = config.control.pid.Kp;
model_params.pid_Ki = config.control.pid.Ki;
model_params.pid_Kd = config.control.pid.Kd;

% Orbital parameters
model_params.orbit_period = config.orbit.period;
model_params.orbit_altitude = config.orbit.altitude;

fprintf('Simulink model parameters loaded.\n');
fprintf('Open the Simulink models:\n');
fprintf('  - adcs_full_system.slx\n');
fprintf('  - adcs_sensors.slx\n');
fprintf('  - adcs_controllers.slx\n');
fprintf('  - adcs_actuators.slx\n');
