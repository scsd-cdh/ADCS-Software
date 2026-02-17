# SIMULINK Models Directory

This directory contains SIMULINK models for the ADCS system.

## Models

### adcs_full_system.slx
Complete ADCS system simulation including:
- Spacecraft dynamics
- Sensor models
- Attitude determination
- Control algorithms
- Actuator models

### Subsystem Models

- **adcs_sensors.slx**: Sensor subsystem (magnetometer, sun sensor, gyroscope)
- **adcs_controllers.slx**: Control algorithms (B-dot, PID, reaction wheel control)
- **adcs_actuators.slx**: Actuator models (magnetorquers, reaction wheels)
- **adcs_dynamics.slx**: Spacecraft attitude dynamics

## Usage

1. Run the setup script to load parameters:
   ```matlab
   run('setup_simulink_models.m')
   ```

2. Open the desired SIMULINK model:
   ```matlab
   open_system('adcs_full_system.slx')
   ```

3. Run the simulation using the Simulink UI or programmatically:
   ```matlab
   sim('adcs_full_system.slx')
   ```

## Model Parameters

All model parameters are defined in `setup_simulink_models.m` and loaded from the main configuration file `adcs_config.m`.

## Code Generation

These models can be used with MATLAB Coder and Simulink Coder to generate C/C++ code for embedded systems:

```matlab
% Configure for code generation
set_param('adcs_full_system', 'SystemTargetFile', 'ert.tlc');

% Generate code
slbuild('adcs_full_system');
```

## Notes

- Ensure all MATLAB function blocks reference the correct .m files in the src/ directory
- Models are designed for fixed-step simulation with sample time defined in config
- For hardware-in-the-loop testing, use the Real-Time Workshop target
