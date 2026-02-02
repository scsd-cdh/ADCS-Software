function [r_eci, v_eci] = propagate_orbit(r0, v0, dt, mu)
% PROPAGATE_ORBIT Simple orbital propagation using two-body dynamics
%   [r_eci, v_eci] = PROPAGATE_ORBIT(r0, v0, dt, mu)
%   
%   Inputs:
%       r0  - Initial position vector in ECI frame (3x1) [m]
%       v0  - Initial velocity vector in ECI frame (3x1) [m/s]
%       dt  - Time step [seconds]
%       mu  - Gravitational parameter [m^3/s^2]
%   
%   Outputs:
%       r_eci - Final position vector (3x1) [m]
%       v_eci - Final velocity vector (3x1) [m/s]

    % Simple Runge-Kutta 4th order integration
    [r_eci, v_eci] = rk4_orbit(r0, v0, dt, mu);
end

function [r, v] = rk4_orbit(r0, v0, dt, mu)
% RK4_ORBIT Runge-Kutta 4th order orbital propagation
    
    % k1
    [a1] = orbital_accel(r0, mu);
    k1_r = v0;
    k1_v = a1;
    
    % k2
    r2 = r0 + 0.5*dt*k1_r;
    v2 = v0 + 0.5*dt*k1_v;
    [a2] = orbital_accel(r2, mu);
    k2_r = v2;
    k2_v = a2;
    
    % k3
    r3 = r0 + 0.5*dt*k2_r;
    v3 = v0 + 0.5*dt*k2_v;
    [a3] = orbital_accel(r3, mu);
    k3_r = v3;
    k3_v = a3;
    
    % k4
    r4 = r0 + dt*k3_r;
    v4 = v0 + dt*k3_v;
    [a4] = orbital_accel(r4, mu);
    k4_r = v4;
    k4_v = a4;
    
    % Update
    r = r0 + (dt/6)*(k1_r + 2*k2_r + 2*k3_r + k4_r);
    v = v0 + (dt/6)*(k1_v + 2*k2_v + 2*k3_v + k4_v);
end

function a = orbital_accel(r, mu)
% ORBITAL_ACCEL Compute orbital acceleration
    r_mag = norm(r);
    a = -mu * r / (r_mag^3);
end
