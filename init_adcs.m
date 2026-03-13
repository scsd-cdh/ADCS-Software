% ADCS System Initialization Script
% Initializes the MATLAB workspace for ADCS simulations

clear all;
close all;
clc;

fprintf('Initializing ADCS System...\n');

%% Add paths
fprintf('Adding ADCS paths to MATLAB path...\n');
adcs_root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(adcs_root, 'src')));
addpath(genpath(fullfile(adcs_root, 'config')));
addpath(genpath(fullfile(adcs_root, 'simulink')));
addpath(genpath(fullfile(adcs_root, 'examples')));

fprintf('  - Attitude determination algorithms\n');
fprintf('  - Attitude control algorithms\n');
fprintf('  - Sensor models\n');
fprintf('  - Actuator models\n');
fprintf('  - Utility functions\n');

%% Load configuration
fprintf('\nLoading ADCS configuration...\n');
config = adcs_config();
fprintf('  - Spacecraft mass: %.2f kg\n', config.spacecraft.mass);
fprintf('  - Orbit altitude: %.1f km\n', config.orbit.altitude/1000);
fprintf('  - Simulation duration: %.1f minutes\n', config.simulation.duration/60);

%% Set constants
fprintf('\nSetting physical constants...\n');
constants.mu_earth = 3.986004418e14;  % [m^3/s^2]
constants.earth_radius = 6371e3;  % [m]
constants.G = 6.67430e-11;  % [m^3/(kg·s^2)]

%% Initialize random seed for repeatability
rng(0);

fprintf('\nADCS System initialized successfully!\n');
fprintf('Type "help <function_name>" for documentation on any function.\n\n');
fprintf('Example scripts:\n');
fprintf('  - run_examples/detumbling_simulation.m\n');
fprintf('  - run_examples/pointing_simulation.m\n');
fprintf('  - run_examples/attitude_estimation.m\n');
