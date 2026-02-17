function B_eci = earth_magnetic_field(r_eci, time)
% EARTH_MAGNETIC_FIELD Simplified Earth magnetic field model (dipole)
%   B_eci = EARTH_MAGNETIC_FIELD(r_eci, time)
%   
%   Inputs:
%       r_eci - Position vector in ECI frame (3x1) [m]
%       time  - Current time [seconds]
%   
%   Output:
%       B_eci - Magnetic field vector in ECI frame (3x1) [Tesla]
%
%   Note: This is a simplified dipole model. For higher accuracy,
%         use IGRF (International Geomagnetic Reference Field) model.

    % Earth magnetic dipole moment
    M_earth = 7.96e15;  % [A·m^2]
    
    % Earth's magnetic pole (approximate, in ECI frame)
    % The magnetic pole rotates with Earth
    theta = 2*pi * time / 86400;  % Earth rotation
    m_hat = [sin(11.5*pi/180)*cos(theta); 
             sin(11.5*pi/180)*sin(theta); 
             cos(11.5*pi/180)];  % Tilted dipole axis
    
    % Position magnitude
    r = norm(r_eci);
    r_hat = r_eci / r;
    
    % Dipole field equation
    % B = (mu_0 / (4*pi*r^3)) * (3*(m·r_hat)*r_hat - m)
    mu_0 = 4*pi*1e-7;  % [H/m]
    
    m_dot_r = dot(m_hat, r_hat);
    B_eci = (mu_0 * M_earth / (4*pi*r^3)) * (3*m_dot_r*r_hat - m_hat);
end
