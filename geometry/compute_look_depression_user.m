% =========================================================================
% COMPUTE LOOK/DEPRESSION ANGLES (User-Provided Method)
%
% Description:
%   Calculates Look Angle (Azimuth) and Depression Angle (Elevation) of a
%   reference point (radar) relative to the aircraft's body coordinate
%   system (X-fwd, Y-right, Z-down).
%   This implementation uses the specific coordinate transformation sequence
%   provided by the user from a previous project. Assumes WGS-84 ellipsoid.
%
% Inputs:
%   radar_lat_deg (double): Latitude of the reference radar site (degrees).
%   radar_lon_deg (double): Longitude of the reference radar site (degrees).
%   radar_alt_m (double): Altitude (Height above ellipsoid) of the radar site (meters).
%   ac_lat_deg (double): Current latitude of the aircraft (degrees).
%   ac_lon_deg (double): Current longitude of the aircraft (degrees).
%   ac_alt_m (double): Current altitude (Height above ellipsoid) of the aircraft (meters).
%   ac_roll_deg (double): Current roll angle of the aircraft (degrees, +ve right wing down).
%   ac_pitch_deg (double): Current pitch angle of the aircraft (degrees, +ve nose up).
%   ac_heading_deg (double): Current true heading of the aircraft (degrees, 0=N, 90=E).
%   earth (struct): Earth model object (passed but WGS-84 constants are hardcoded below).
%
% Outputs:
%   look_angle_deg (double): Look angle (Azimuth) from aircraft nose to radar (degrees).
%                            Range [0, 360) or [-180, 180), depending on atan2 usage.
%                            Here, it's atan2(Y, X) -> [-180, 180]. +ve is right of nose.
%   depr_angle_deg (double): Depression angle (Elevation) from aircraft XY plane to radar (degrees).
%                            Range [-90, 90]. +ve is below aircraft XY plane.
%
% Based on user-provided code snippet.
% =========================================================================
function [look_angle_deg, depr_angle_deg] = compute_look_depression_user(radar_lat_deg, radar_lon_deg, radar_alt_m, ac_lat_deg, ac_lon_deg, ac_alt_m, ac_roll_deg, ac_pitch_deg, ac_heading_deg, earth)

    % --- Call user's sequence of functions ---

    % 1. Convert LLH to ECEF (User's EFG) for both radar and aircraft
    [E_radar, F_radar, G_radar] = LLH_2_EFG_local(radar_lat_deg, radar_lon_deg, radar_alt_m);
    [E_ac, F_ac, G_ac] = LLH_2_EFG_local(ac_lat_deg, ac_lon_deg, ac_alt_m);

    % 2. Convert ECEF vector (AC relative to Radar) to local ENU frame at Radar site
    [e_enu, n_enu, u_enu] = EFG_to_ENU_local(radar_lat_deg, radar_lon_deg, E_radar, F_radar, G_radar, E_ac, F_ac, G_ac);

    % 3. Convert ENU vector to NED vector (still local frame at Radar site)
    [n_ned, e_ned, d_ned] = ENU_to_NED_local(e_enu, n_enu, u_enu);

    % 4. Convert the NED vector (representing LOS from Radar to AC) into the Aircraft's Body Frame (XYZ)
    % Note: This transforms the vector pointing *from* the radar *to* the aircraft,
    % expressed in the radar's NED frame, into the aircraft's body frame.
    [X_radar_in_ac_body, Y_radar_in_ac_body, Z_radar_in_ac_body] = NED_to_XYZ_local(n_ned, e_ned, d_ned, ac_roll_deg, ac_pitch_deg, ac_heading_deg);

    % 5. Calculate Look/Depression angles from the aircraft body frame towards the radar position
    % We want the angles *to* the radar, so we need the vector from AC to Radar in AC body frame.
    % This is the negative of the vector calculated in step 4.
    X_ac_to_radar = -X_radar_in_ac_body;
    Y_ac_to_radar = -Y_radar_in_ac_body;
    Z_ac_to_radar = -Z_radar_in_ac_body;

    [look_angle_deg, depr_angle_deg] = XYZ_to_LDR_local(X_ac_to_radar, Y_ac_to_radar, Z_ac_to_radar);

end % END MAIN FUNCTION

% =========================================================================
% USER-PROVIDED HELPER FUNCTIONS (Copied Directly)
% Made local by appending "_local" to avoid potential conflicts
% =========================================================================

function [E, F, G] = LLH_2_EFG_local(lat_dd, long_dd, height_m)
    lat_rad = deg2rad(lat_dd);
    long_rad = deg2rad(long_dd);

    % Ref Ellipsoid Datum WGS84 (Hardcoded constants)
    a = 6378137; % meters, semi major axis
    b = 6356752.314245; % meters, semi minor axis
    e_sq = 1 - (b^2 / a^2); % Eccentricity squared directly

    v = a ./ sqrt(1 - e_sq .* sin(lat_rad).^2); % Radius of curvature in prime vertical (N)
    E = (v + height_m) .* cos(lat_rad) .* cos(long_rad); % ECEF X
    F = (v + height_m) .* cos(lat_rad) .* sin(long_rad); % ECEF Y
    G = (v .* (1 - e_sq) + height_m) .* sin(lat_rad);    % ECEF Z
end

function [e, n, u] = EFG_to_ENU_local(lat0_dd, long0_dd, E0, F0, G0, E, F, G)
    % Calculates ENU coordinates of point (E,F,G) relative to origin (E0,F0,G0)
    % with the local frame defined at (lat0_dd, long0_dd).
    lat0_rad = deg2rad(lat0_dd);
    long0_rad = deg2rad(long0_dd);

    % Vector components in ECEF from origin to point
    dE = E - E0;
    dF = F - F0;
    dG = G - G0;

    % Rotation from ECEF to ENU
    e = -dE * sin(long0_rad) + dF * cos(long0_rad);
    n = -dE * sin(lat0_rad) * cos(long0_rad) - dF * sin(lat0_rad) * sin(long0_rad) + dG * cos(lat0_rad);
    u =  dE * cos(lat0_rad) * cos(long0_rad) + dF * cos(lat0_rad) * sin(long0_rad) + dG * sin(lat0_rad);
end

function [n, e, d] = ENU_to_NED_local(e_in, n_in, u_in)
    % Simple coordinate swap and negation for ENU -> NED
    n = n_in;
    e = e_in;
    d = -u_in;
end

function [X_body, Y_body, Z_body] = NED_to_XYZ_local(n_ned, e_ned, d_ned, roll_deg, pitch_deg, heading_deg)
    % Transforms a vector from the local NED frame (at aircraft location)
    % to the aircraft's Body frame (X-fwd, Y-right, Z-down).
    % Inputs: n, e, d components of the vector in NED.
    %         Aircraft attitude angles (roll=mu, pitch=theta, heading=psi) in degrees.

    % Convert attitude angles to radians
    mu_rad    = deg2rad(roll_deg);    % Roll
    theta_rad = deg2rad(pitch_deg);   % Pitch
    psi_rad   = deg2rad(heading_deg); % Heading (Yaw)

    % Pre-calculate sines and cosines
    c_mu = cos(mu_rad); s_mu = sin(mu_rad);
    c_th = cos(theta_rad); s_th = sin(theta_rad);
    c_psi = cos(psi_rad); s_psi = sin(psi_rad);

    % Rotation Matrix (NED to Body frame - ZYX rotation sequence)
    % Note: This matrix rotates the *vector* from NED to Body.
    % R_ned_to_body =
    %   [ c_th*c_psi,                 c_th*s_psi,                -s_th;
    %     s_mu*s_th*c_psi-c_mu*s_psi, s_mu*s_th*s_psi+c_mu*c_psi, s_mu*c_th;
    %     c_mu*s_th*c_psi+s_mu*s_psi, c_mu*s_th*s_psi-s_mu*c_psi, c_mu*c_th ]

    % Apply rotation: [X; Y; Z]_body = R_ned_to_body * [N; E; D]_ned
    X_body = n_ned * (c_th * c_psi) + e_ned * (c_th * s_psi) - d_ned * (s_th);
    Y_body = n_ned * (s_mu * s_th * c_psi - c_mu * s_psi) + e_ned * (s_mu * s_th * s_psi + c_mu * c_psi) + d_ned * (s_mu * c_th);
    Z_body = n_ned * (c_mu * s_th * c_psi + s_mu * s_psi) + e_ned * (c_mu * s_th * s_psi - s_mu * c_psi) + d_ned * (c_mu * c_th);
end

function [look, depr] = XYZ_to_LDR_local(X_body, Y_body, Z_body)
    % Calculates Look (Azimuth) and Depression (Elevation) angles
    % from aircraft body coordinates (X-fwd, Y-right, Z-down).

    % Look Angle (Azimuth): Angle in the body XY plane, measured from X-axis.
    % atan2(Y, X) gives angle from +X axis, positive towards +Y (right). Range [-pi, pi].
    look_rad = atan2(Y_body, X_body);

    % Depression Angle (Elevation): Angle from the body XY plane.
    % Positive angle means the target is below the aircraft's XY plane (Z is positive Down).
    % atan2(-Z, sqrt(X^2+Y^2)) gives angle from XY plane, positive is Down. Range [-pi/2, pi/2].
    depr_rad = atan2(-Z_body, sqrt(X_body^2 + Y_body^2)); % User code had -atan2(Z, sqrt(...)) which is equivalent

    % Convert to degrees
    look = rad2deg(look_rad); % Range [-180, 180]
    depr = rad2deg(depr_rad); % Range [-90, 90]
end