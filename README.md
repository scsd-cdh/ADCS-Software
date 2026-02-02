# ADCS-Software
SC-FREYRS Attitude Determination and Control System

## Overview

This repository contains MATLAB and SIMULINK simulations and generated code for a CubeSat Attitude Determination and Control System (ADCS). The software provides a complete framework for:

- **Attitude Determination**: Sensor models, TRIAD algorithm, Extended Kalman Filter
- **Attitude Control**: B-dot detumbling, PID control, reaction wheel control
- **Sensor Simulation**: Magnetometer, sun sensor, gyroscope models
- **Actuator Simulation**: Magnetorquer and reaction wheel models
- **Complete System Simulation**: SIMULINK models for full ADCS system

## Directory Structure

```
ADCS-Software/
├── src/
│   ├── attitude_determination/    # Attitude estimation algorithms
│   │   ├── triad_algorithm.m
│   │   └── ekf_attitude_update.m
│   ├── attitude_control/          # Control algorithms
│   │   ├── bdot_controller.m
│   │   ├── pid_attitude_controller.m
│   │   ├── magnetorquer_control.m
│   │   └── reaction_wheel_control.m
│   ├── sensors/                   # Sensor models
│   │   ├── magnetometer_model.m
│   │   ├── sun_sensor_model.m
│   │   └── gyroscope_model.m
│   ├── actuators/                 # Actuator models
│   │   ├── magnetorquer_actuator.m
│   │   └── reaction_wheel_actuator.m
│   └── utils/                     # Utility functions
│       ├── quat_mult.m
│       ├── quat_conj.m
│       ├── quat_to_dcm.m
│       ├── euler_to_quat.m
│       └── quat_to_euler.m
├── simulink/                      # SIMULINK models
│   ├── setup_simulink_models.m
│   └── README.md
├── config/                        # Configuration files
│   └── adcs_config.m
├── examples/                      # Example simulations
│   ├── detumbling_simulation.m
│   ├── pointing_simulation.m
│   └── attitude_estimation.m
└── init_adcs.m                   # Initialization script

```

## Getting Started

### Prerequisites

- MATLAB R2018b or later
- Simulink (optional, for SIMULINK models)
- Aerospace Toolbox (recommended)

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/scsd-cdh/ADCS-Software.git
   cd ADCS-Software
   ```

2. Initialize the ADCS system in MATLAB:
   ```matlab
   init_adcs
   ```

### Quick Start

Run one of the example simulations:

```matlab
% Detumbling simulation using B-dot control
run('examples/detumbling_simulation.m')

% Attitude pointing using PID control
run('examples/pointing_simulation.m')

% Attitude estimation using TRIAD
run('examples/attitude_estimation.m')
```

## Features

### Attitude Determination

- **TRIAD Algorithm**: Deterministic attitude determination from two vector measurements
- **Extended Kalman Filter**: Optimal attitude estimation with gyroscope integration
- **Sensor Models**: Realistic magnetometer, sun sensor, and gyroscope simulations

### Attitude Control

- **B-dot Detumbling**: Magnetic control for initial angular velocity reduction
- **PID Controller**: Three-axis attitude pointing control
- **Reaction Wheel Control**: High-precision attitude control using momentum exchange
- **Magnetorquer Control**: Magnetic torque generation for momentum management

### Utility Functions

- Quaternion mathematics (multiplication, conjugate, conversions)
- Direction Cosine Matrix (DCM) conversions
- Euler angle conversions

## Configuration

System parameters are defined in `config/adcs_config.m`:

- Spacecraft physical properties (mass, inertia)
- Sensor specifications (noise, bias, sampling rates)
- Actuator limits (torque, dipole moment)
- Control gains (PID, B-dot)
- Orbital parameters
- Simulation settings

Modify this file to customize the ADCS for your specific CubeSat.

## SIMULINK Models

The `simulink/` directory contains SIMULINK models for system-level simulation:

1. Load model parameters:
   ```matlab
   cd simulink
   run('setup_simulink_models.m')
   ```

2. Open and run SIMULINK models for:
   - Complete ADCS system simulation
   - Individual subsystem testing
   - Hardware-in-the-loop integration

## Code Generation

MATLAB and SIMULINK code can be compiled for embedded systems:

```matlab
% Generate C code from MATLAB functions
codegen function_name -args {arg_types}

% Generate C code from SIMULINK models
slbuild('model_name')
```

## Documentation

Each MATLAB function includes comprehensive documentation. View help for any function:

```matlab
help triad_algorithm
help pid_attitude_controller
help magnetometer_model
```

## Examples

### Detumbling Simulation
Simulates a CubeSat with high initial angular velocity being stabilized using B-dot control with magnetorquers.

### Pointing Simulation
Demonstrates three-axis attitude control to point the spacecraft to a desired orientation using PID control.

### Attitude Estimation
Shows how to estimate spacecraft attitude from magnetometer and sun sensor measurements using the TRIAD algorithm.

## Contributing

Contributions are welcome! Please follow these guidelines:
1. Fork the repository
2. Create a feature branch
3. Make your changes with clear comments
4. Test thoroughly
5. Submit a pull request

## References

- Wertz, J. R. (Editor), "Spacecraft Attitude Determination and Control", Kluwer Academic Publishers, 1978
- Sidi, M. J., "Spacecraft Dynamics and Control", Cambridge University Press, 1997
- Wie, B., "Space Vehicle Dynamics and Control", AIAA Education Series, 2008

## License

This project is developed for educational and research purposes.

## Contact

For questions or support, please open an issue on GitHub or contact the development team.
