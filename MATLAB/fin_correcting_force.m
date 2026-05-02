function [M_body, F_body, x_cp_fins, static_margin, CNa_total] = fin_correcting_force( ...
    V_body, ang_rates, rho, x_cg)
%FIN_CORRECTING_FORCE  Passive fin correcting forces and moments (all 4 fins).
%
%   Calculates the total restoring forces and moments in the body frame
%   (roll, pitch, yaw) produced by 4 passive fins placed 90 degrees apart.
%   Uses the Barrowman method with pitch damping.
%
%   Fin layout (looking from behind, +Y = right, +Z = down):
%     Fin 1:   0 deg (top)     — responds to pitch (alpha)
%     Fin 2:  90 deg (right)   — responds to yaw   (beta)
%     Fin 3: 180 deg (bottom)  — responds to pitch (alpha)
%     Fin 4: 270 deg (left)    — responds to yaw   (beta)
%
%   This function is compatible with MATLAB Function blocks in Simulink.
%
%   INPUTS
%     V_body     - 3x1 velocity in body frame [u; v; w] [m/s]
%                  u = forward, v = right, w = down
%     ang_rates  - 3x1 angular rates [p; q; r] (roll, pitch, yaw) [rad/s]
%     rho        - Air density [kg/m^3]
%     x_cg       - Centre of gravity position from nose tip [m]
%
%   OUTPUTS
%     M_body       - 3x1 correcting moments [roll; pitch; yaw] [N*m]
%                    Includes aerodynamic damping.
%     F_body       - 3x1 correcting forces [Fx; Fy; Fz] in body frame [N]
%     x_cp_fins    - Fin CP location from nose tip [m]
%     static_margin - Static margin in calibres (positive = stable)
%     CNa_total    - Total normal force coeff gradient (all fins) [1/rad]
%
%   Reference: Barrowman, J.S. (1967) — The Practical Calculation of the
%   Aerodynamic Characteristics of Slender Finned Vehicles.

%#codegen
    coder.extrinsic('warning', 'get_param', 'bdroot');

    % --- Force inputs to be real scalar doubles ---
    u = double(V_body(1));          % forward velocity [m/s]
    v = double(V_body(2));          % lateral velocity (right) [m/s]
    w = double(V_body(3));          % vertical velocity (down) [m/s]

    p = double(ang_rates(1));       % roll rate  [rad/s]
    q_rate = double(ang_rates(2));  % pitch rate [rad/s]
    r_rate = double(ang_rates(3));  % yaw rate   [rad/s]

    rho  = double(rho(1));
    x_cg = double(x_cg(1));

    % --- Compute airspeed, alpha, beta from body-frame velocity ---
    airspeed = sqrt(u^2 + v^2 + w^2);
    V_MIN = 0.1;   % minimum airspeed to avoid division by zero [m/s]

    % --- Pre-initialise outputs for codegen ---
    M_body        = zeros(3,1);
    F_body        = zeros(3,1);
    x_cp_fins     = 0.0;
    static_margin = 0.0;
    CNa_total     = 0.0;

    % --- Guard: airspeed too low for meaningful aero ---
    if airspeed < V_MIN
        return;
    end

    % --- Angle of attack and sideslip from body-frame velocity ---
    %   Only compute if forward velocity is positive (rocket moving forward)
    if u <= 0.0
        return;
    end
    alpha = atan2(w, u);            % AoA [rad]
    beta  = asin(v / airspeed);     % sideslip [rad]

    % --- Dead zone: ignore angles below 0.1 deg (numerical noise) ---
    ALPHA_MIN = 0.00175;            % ~0.1 deg in rad
    if abs(alpha) < ALPHA_MIN
        alpha = 0.0;
    end
    if abs(beta) < ALPHA_MIN
        beta = 0.0;
    end

    % --- Warning: Barrowman is inaccurate beyond ~15 deg AoA ---
    AOA_WARN = 15.0 * pi / 180.0;      % 15 deg in rad
    if abs(alpha) > AOA_WARN || abs(beta) > AOA_WARN
        t = get_param(bdroot, 'SimulationTime');
        warning('fin_correcting_force [t=%.4fs]: AoA exceeds 15 deg (alpha=%.1f deg, beta=%.1f deg). Results may be inaccurate.', ...
            t, alpha * 180.0 / pi, beta * 180.0 / pi);
    end

    % --- Clamp airspeed for damping denominator ---
    V_safe = max(airspeed, V_MIN);

    % --- Rocket geometry constants ---
    body_d   = 0.065;       % Body tube diameter [m]

    % --- Fin geometry constants ---
    cr        = 0.11;       % Fin root chord [m]
    ct        = 0.061;      % Fin tip chord [m]
    semi_span = 0.055;      % Fin semi-span (from body surface) [m]
    sweep_le  = 53.0;       % Fin leading-edge sweep angle [deg]
    x_fin_le  = 0.475;      % Fin leading edge position from nose tip [m]

    % --- Derived geometry ---
    rad = body_d / 2.0;
    S_ref = pi * rad^2;

    % --- Sweep distance ---
    x_sweep = semi_span * tand(sweep_le);

    % --- Chordwise CP of fin (Barrowman, from fin root LE) ---
    x_f = (x_sweep * (cr + 2.0 * ct)) / (3.0 * (cr + ct)) + ...
          (1.0 / 6.0) * (cr + ct - (cr * ct) / (cr + ct));

    % --- Fin CP from nose tip ---
    x_cp_fins = x_fin_le + x_f;

    % --- Moment arm from CG to fin CP ---
    moment_arm = x_cp_fins - x_cg;

    % --- Static margin in calibres ---
    static_margin = moment_arm / body_d;

    % --- Normal force coefficient gradient (single fin, per radian) ---
    CNa_1fin = (4.0 * (semi_span / body_d)^2) / ...
               (1.0 + sqrt(1.0 + (2.0 * semi_span / (cr + ct))^2));

    % --- Fin-body interference factors ---
    K_fb = 1.0 + rad / (semi_span + rad);
    K_bf = rad / (semi_span + rad);
    CNa_1fin_eff = CNa_1fin * (K_fb + K_bf);
    CNa_total    = 4.0 * CNa_1fin_eff;

    % --- Dynamic pressure ---
    q = 0.5 * rho * airspeed^2;

    % =====================================================================
    %   Per-fin effective AoA
    %
    %   Fin positions (clock angles looking aft):
    %     Fin 1:   0 deg — top      — normal force acts in -Z (pitch plane)
    %     Fin 2:  90 deg — right    — normal force acts in +Y (yaw plane)
    %     Fin 3: 180 deg — bottom   — normal force acts in +Z (pitch plane)
    %     Fin 4: 270 deg — left     — normal force acts in -Y (yaw plane)
    %
    %   Effective AoA for each fin:
    %     aoa_eff = alpha * cos(fin_angle) + beta * sin(fin_angle)
    % =====================================================================

    % Fin 1 (0 deg):   aoa_eff =  alpha
    % Fin 3 (180 deg): aoa_eff = -alpha
    % Fin 2 (90 deg):  aoa_eff =  beta
    % Fin 4 (270 deg): aoa_eff = -beta

    aoa_fin1 =  alpha;
    aoa_fin2 =  beta;
    aoa_fin3 = -alpha;
    aoa_fin4 = -beta;

    % --- Normal force per fin (positive = away from body in fin's plane) ---
    F1 = q * S_ref * CNa_1fin_eff * aoa_fin1;
    F2 = q * S_ref * CNa_1fin_eff * aoa_fin2;
    F3 = q * S_ref * CNa_1fin_eff * aoa_fin3;
    F4 = q * S_ref * CNa_1fin_eff * aoa_fin4;

    % =====================================================================
    %   Resolve fin forces into body frame
    %
    %   Body frame: X = forward, Y = right, Z = down
    %
    %   Fin 1 (top):    force in -Z direction  →  Fz -= F1
    %   Fin 3 (bottom): force in +Z direction  →  Fz += F3
    %   Fin 2 (right):  force in +Y direction  →  Fy += F2
    %   Fin 4 (left):   force in -Y direction  →  Fy -= F4
    % =====================================================================

    Fy = F2 - F4;          % net lateral force (yaw plane)
    Fz = F3 - F1;          % net vertical force (pitch plane)

    F_body(1) = 0.0;       % no axial force from passive fins at small AoA
    F_body(2) = Fy;
    F_body(3) = Fz;

    % =====================================================================
    %   Correcting moments about CG
    %
    %   Pitch moment (about Y-axis): driven by Fz and moment arm
    %     Positive Fz (downward) with CP behind CG → nose-down → restoring
    %   Yaw moment (about Z-axis): driven by Fy and moment arm
    %     Positive Fy (right) with CP behind CG → nose-left → restoring
    % =====================================================================

    M_pitch_aero = -Fz * moment_arm;   % restoring pitch moment
    M_yaw_aero   = -Fy * moment_arm;   % restoring yaw moment

    % --- Roll moment from fin cant (zero for passive fins) ---
    M_roll_aero = 0.0;

    % =====================================================================
    %   Aerodynamic damping moments (oppose angular rates)
    %
    %   Non-dimensional rate: q_hat = omega * d / (2*V)
    %   Cmq ≈ -2 * CNa_1fin_eff * (moment_arm/d)^2  per fin pair
    % =====================================================================

    Cmq = -2.0 * CNa_1fin_eff * (moment_arm / body_d)^2;
    damp_factor = q * S_ref * body_d * (body_d / (2.0 * V_safe));

    % Two fins contribute to pitch damping, two to yaw damping
    M_pitch_damp = 2.0 * Cmq * damp_factor * q_rate;
    M_yaw_damp   = 2.0 * Cmq * damp_factor * r_rate;

    % Roll damping (from all 4 fins resisting spin)
    %   Clp ≈ -CNa_1fin_eff * (r + y_cp)^2 / (S_ref * d) per fin
    y_cp = (semi_span / 3.0) * (cr + 2.0 * ct) / (cr + ct);
    roll_arm = rad + y_cp;
    Clp = -4.0 * CNa_1fin_eff * roll_arm^2 / (S_ref * body_d);
    M_roll_damp = Clp * damp_factor * p;

    % --- Total moments ---
    M_body(1) = M_roll_aero  + M_roll_damp;    % roll
    M_body(2) = M_pitch_aero + M_pitch_damp;   % pitch
    M_body(3) = M_yaw_aero   + M_yaw_damp;     % yaw

end
