function [F_normal, F_roll, F_axial, tau_roll, alpha_eff, q] = fin_force_barrowman( ...
    aoa, cant_angle, airspeed, rho)
%FIN_FORCE_BARROWMAN  Aerodynamic forces on a single fin (Barrowman method).
%
%   Calculates normal force, roll force, axial force, and roll torque for
%   one trapezoidal fin using the Barrowman normal-force-coefficient method
%   with fin-body and body-carryover interference factors.
%
%   This function is compatible with MATLAB Function blocks in Simulink.
%
%   INPUTS
%     aoa          - Rocket angle of attack [deg]
%     cant_angle   - Fin cant angle [deg]
%     airspeed     - Freestream airspeed [m/s]
%     rho          - Air density [kg/m^3]
%
%   OUTPUTS
%     F_normal   - Normal force on fin [N]
%     F_roll     - Roll (circumferential) force component [N]
%     F_axial    - Axial (in-plane) force component [N]
%     tau_roll   - Roll torque about body centreline [N*m]
%     alpha_eff  - Effective angle of attack [rad]
%     q          - Dynamic pressure [Pa]
%
%   Reference: Barrowman, J.S. (1967) — The Practical Calculation of the
%   Aerodynamic Characteristics of Slender Finned Vehicles.

%#codegen

    % --- Force inputs and outputs to be real scalar doubles ---
    aoa       = double(aoa(1));
    cant_angle = double(cant_angle(1));
    airspeed  = double(airspeed(1));
    rho       = double(rho(1));

    F_normal  = 0.0;
    F_roll    = 0.0;
    F_axial   = 0.0;
    tau_roll  = 0.0;
    alpha_eff = 0.0;
    q         = 0.0;

    % --- Zero outputs for negative airspeed (physically invalid) ---
    if airspeed < 0.0
        return;
    end

    % --- Fin geometry constants ---
    cr         = 0.11;     % Fin root chord [m]
    ct         = 0.061;    % Fin tip chord [m]
    semi_span  = 0.055;    % Fin semi-span (from body surface) [m]
    sweep_angle = 53.0;    % Fin leading-edge sweep angle [deg]  %#ok<NASGU>
    cfd_scale  = 1.0;      % CFD correction scale factor [-]
    body_d     = 0.065;    % Body tube diameter [m]

    % --- Reference area (body cross-section) ---
    r = body_d / 2.0;
    S_ref = pi * r^2;

    % --- Normal force coefficient gradient (single fin, per radian) ---
    %   CNa_base = 4*(s/d)^2 / [1 + sqrt(1 + (2*s/(cr+ct))^2)]
    CNa_base = (4.0 * (semi_span / body_d)^2) / ...
               (1.0 + sqrt(1.0 + (2.0 * semi_span / (cr + ct))^2));

    % Fin-body interference factors
    K_fb = 1.0 + r / (semi_span + r);      % fin-on-body
    K_bf = r / (semi_span + r);             % body carryover
    CNa  = CNa_base * (K_fb + K_bf);

    % --- Spanwise centre of pressure (from body surface) ---
    y_cp = (semi_span / 3.0) * (cr + 2.0 * ct) / (cr + ct);

    % --- Roll moment arm (from centreline) ---
    arm = r + y_cp;

    % --- Effective angle of attack ---
    cant_rad  = cant_angle * pi / 180.0;
    aoa_rad   = aoa * pi / 180.0;
    alpha_eff = aoa_rad + cant_rad;

    % --- Dynamic pressure ---
    q = 0.5 * rho * airspeed^2;

    % --- Forces ---
    F_normal = q * S_ref * CNa * alpha_eff * cfd_scale;
    F_roll   = F_normal * sin(cant_rad);
    F_axial  = F_normal * cos(cant_rad);
    tau_roll = F_roll * arm;

end
