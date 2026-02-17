% Test Script for ADCS Functions
% Verifies that core ADCS functions are working correctly

clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('ADCS Functions Test Suite\n');
fprintf('========================================\n\n');

test_passed = 0;
test_failed = 0;

%% Test 1: Quaternion Multiplication
fprintf('Test 1: Quaternion Multiplication... ');
try
    q1 = [1; 0; 0; 0];
    q2 = [cos(pi/4); 0; 0; sin(pi/4)];  % 45-degree rotation about z-axis
    q_result = quat_mult(q1, q2);
    
    % Result should be approximately q2
    if norm(q_result - q2) < 1e-10
        fprintf('PASSED\n');
        test_passed = test_passed + 1;
    else
        fprintf('FAILED\n');
        test_failed = test_failed + 1;
    end
catch e
    fprintf('ERROR: %s\n', e.message);
    test_failed = test_failed + 1;
end

%% Test 2: Quaternion to DCM Conversion
fprintf('Test 2: Quaternion to DCM... ');
try
    q = [1; 0; 0; 0];  % Identity quaternion
    DCM = quat_to_dcm(q);
    
    % Should return identity matrix
    if norm(DCM - eye(3)) < 1e-10
        fprintf('PASSED\n');
        test_passed = test_passed + 1;
    else
        fprintf('FAILED\n');
        test_failed = test_failed + 1;
    end
catch e
    fprintf('ERROR: %s\n', e.message);
    test_failed = test_failed + 1;
end

%% Test 3: Euler to Quaternion Conversion
fprintf('Test 3: Euler to Quaternion... ');
try
    roll = 0;
    pitch = 0;
    yaw = 0;
    q = euler_to_quat(roll, pitch, yaw);
    
    % Should return identity quaternion
    if norm(q - [1; 0; 0; 0]) < 1e-10
        fprintf('PASSED\n');
        test_passed = test_passed + 1;
    else
        fprintf('FAILED\n');
        test_failed = test_failed + 1;
    end
catch e
    fprintf('ERROR: %s\n', e.message);
    test_failed = test_failed + 1;
end

%% Test 4: Magnetometer Model
fprintf('Test 4: Magnetometer Model... ');
try
    B_eci = [0; 0; 2e-5];  % Magnetic field in ECI
    DCM = eye(3);  % No rotation
    B_body = magnetometer_model(B_eci, DCM, 0, [0; 0; 0]);
    
    % Should be approximately the same (no rotation, no noise)
    if norm(B_body - B_eci) < 1e-10
        fprintf('PASSED\n');
        test_passed = test_passed + 1;
    else
        fprintf('FAILED\n');
        test_failed = test_failed + 1;
    end
catch e
    fprintf('ERROR: %s\n', e.message);
    test_failed = test_failed + 1;
end

%% Test 5: B-dot Controller
fprintf('Test 5: B-dot Controller... ');
try
    B_body = [1e-5; 0; 2e-5];
    B_body_prev = [0.5e-5; 0; 2e-5];
    dt = 0.1;
    k_bdot = 1e5;
    
    M_cmd = bdot_controller(B_body, B_body_prev, dt, k_bdot);
    
    % Should return a 3x1 vector
    if length(M_cmd) == 3
        fprintf('PASSED\n');
        test_passed = test_passed + 1;
    else
        fprintf('FAILED\n');
        test_failed = test_failed + 1;
    end
catch e
    fprintf('ERROR: %s\n', e.message);
    test_failed = test_failed + 1;
end

%% Test 6: TRIAD Algorithm
fprintf('Test 6: TRIAD Algorithm... ');
try
    % Same vectors in both frames (identity rotation)
    v1_body = [1; 0; 0];
    v1_ref = [1; 0; 0];
    v2_body = [0; 1; 0];
    v2_ref = [0; 1; 0];
    
    q = triad_algorithm(v1_body, v1_ref, v2_body, v2_ref);
    
    % Should return approximately identity quaternion
    if abs(abs(q(1)) - 1) < 0.1  % Allow some tolerance
        fprintf('PASSED\n');
        test_passed = test_passed + 1;
    else
        fprintf('FAILED\n');
        test_failed = test_failed + 1;
    end
catch e
    fprintf('ERROR: %s\n', e.message);
    test_failed = test_failed + 1;
end

%% Test 7: PID Controller
fprintf('Test 7: PID Attitude Controller... ');
try
    q_current = [1; 0; 0; 0];
    q_desired = [cos(0.1); 0; 0; sin(0.1)];  % Small rotation
    omega = [0; 0; 0];
    omega_desired = [0; 0; 0];
    e_integral = [0; 0; 0];
    dt = 0.1;
    Kp = 0.1;
    Ki = 0.01;
    Kd = 0.5;
    I_body = diag([0.02, 0.02, 0.01]);
    
    [tau_cmd, e_integral_new] = pid_attitude_controller(q_current, q_desired, omega, omega_desired, ...
                                                         e_integral, dt, Kp, Ki, Kd, I_body);
    
    % Should return a 3x1 torque vector
    if length(tau_cmd) == 3
        fprintf('PASSED\n');
        test_passed = test_passed + 1;
    else
        fprintf('FAILED\n');
        test_failed = test_failed + 1;
    end
catch e
    fprintf('ERROR: %s\n', e.message);
    test_failed = test_failed + 1;
end

%% Test Summary
fprintf('\n========================================\n');
fprintf('Test Results\n');
fprintf('========================================\n');
fprintf('Tests Passed: %d\n', test_passed);
fprintf('Tests Failed: %d\n', test_failed);
fprintf('Total Tests:  %d\n', test_passed + test_failed);

if test_failed == 0
    fprintf('\nAll tests PASSED! ✓\n');
else
    fprintf('\nSome tests FAILED! ✗\n');
end

fprintf('========================================\n');
