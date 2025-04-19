% =========================================================================
% EARTH MODEL (FINAL - Re-Commented)
% Version: FINAL 1.0
%
% Description:
%   Implements Earth geometry calculations based on the specified model
%   (typically WGS-84 ellipsoid). Provides functions for:
%   - Calculating distance and bearing between points (Haversine).
%   - Updating geographic position based on movement vector (`move`).
%   - Converting between standard coordinate systems:
%     - Geodetic (Latitude, Longitude, Height - LLH)
%     - Earth-Centered, Earth-Fixed (ECEF - Cartesion X,Y,Z from Earth center)
%     - Local Tangent Plane (North-East-Down - NED)
%
% Usage:
%   - Initialized in `main_simulation.m`: `earth = earth_model(config.earth.model);`
%   - Methods called as needed, e.g.:
%     `[dist, bear] = earth.distance_bearing(lat1, lon1, lat2, lon2);`
%     `[X, Y, Z] = earth.LLH_to_ECEF(lat, lon, h);`
%     `R_ned = earth.ECEF_to_NED_matrix(lat, lon);`
%
% Coordinate Systems:
%   - LLH: Geodetic Lat(deg), Lon(deg), Height above ellipsoid (m).
%   - ECEF: Earth-Centered, Earth-Fixed. Cartesian [X, Y, Z] (m).
%           Origin at Earth center. X=PrimeMeridian/Equator, Z=North Pole.
%   - NED: Local Tangent Plane. Cartesian [North, East, Down] (m).
%          Origin at local point (e.g., aircraft position).
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-04]
% =========================================================================
function earth = earth_model(model_type)
    % Input: model_type - String specifying earth model ('wgs84' recommended).

    % Default to WGS84 if no model specified
    if nargin < 1, model_type = 'wgs84'; end

    % Structure to hold model parameters (semi-major axis, flattening, etc.)
    earth_params = struct();

    % --- Select Earth Model Parameters ---
    switch lower(model_type)
        case 'wgs84'
            earth_params.a     = 6378137.0;          % Semi-major axis (meters)
            earth_params.f     = 1/298.257223563;    % Flattening
            earth_params.b     = earth_params.a * (1 - earth_params.f); % Semi-minor axis (meters)
            earth_params.e_sq  = earth_params.f * (2 - earth_params.f); % Eccentricity squared (e^2)
            fprintf('[EARTH] WGS-84 ellipsoid model initialized.\n');
        % Add other models like 'sphere' if needed
        % case 'sphere'
        %     earth_params.a = 6371000.0; % Mean radius
        %     earth_params.f = 0; earth_params.b = earth_params.a; earth_params.e_sq = 0;
        %     fprintf('[EARTH] Spherical model initialized (R=%.0f m).\n', earth_params.a);
        otherwise
            error('Unknown earth model type: %s. Use ''wgs84''.', model_type);
    end
    earth_params.model_type = model_type; % Store model type string

    % --- Assign Public Function Handles ---
    % Makes internal functions callable via the returned 'earth' object.
    % These functions capture the 'earth_params' structure via their definition scope.
    earth.move                = @(lat, lon, dist_m, hdg_deg) move_position_local(earth_params, lat, lon, dist_m, hdg_deg);
    earth.distance_bearing    = @(lat1, lon1, lat2, lon2) distance_bearing_local(earth_params, lat1, lon1, lat2, lon2);
    earth.LLH_to_ECEF         = @(lat, lon, h) LLH_to_ECEF_local(earth_params, lat, lon, h);
    earth.ECEF_to_LLH         = @(X, Y, Z) ECEF_to_LLH_local(earth_params, X, Y, Z); % Accurate inverse
    earth.ECEF_to_NED_matrix  = @(lat, lon) ECEF_to_NED_matrix_local(earth_params, lat, lon); % Rotation matrix ECEF -> NED
    earth.NED_to_ECEF_matrix  = @(lat, lon) NED_to_ECEF_matrix_local(earth_params, lat, lon); % Rotation matrix NED -> ECEF
    earth.params              = earth_params; % Expose parameters if needed externally (read-only concept)

end % END OF FUNCTION earth_model


% =========================================================================
% INTERNAL IMPLEMENTATION FUNCTIONS (Scoped to earth_model)
% =========================================================================

% -------------------------------------------------------------------------
% MOVE POSITION (Spherical Approximation)
% -------------------------------------------------------------------------
function [new_lat, new_lon] = move_position_local(params, lat_deg, lon_deg, distance_m, heading_deg)
    % Updates geographic position based on distance and heading.
    % Uses spherical formulas for simplicity. For very high accuracy over
    % long distances, Vincenty's direct formula on the ellipsoid is better.

    R_earth = params.a; % Use semi-major axis as approximate radius for distance calculation

    % Convert inputs to radians
    lat_rad     = deg2rad(lat_deg);
    lon_rad     = deg2rad(lon_deg);
    heading_rad = deg2rad(heading_deg); % Bearing angle

    % Calculate angular distance traveled
    angular_distance = distance_m / R_earth; % radians

    % Calculate new latitude using spherical law of cosines derivation
    new_lat_rad = asin(sin(lat_rad) * cos(angular_distance) + ...
                       cos(lat_rad) * sin(angular_distance) * cos(heading_rad));

    % Calculate new longitude using spherical law of sines derivation (atan2 handles quadrants)
    new_lon_rad = lon_rad + atan2(sin(heading_rad) * sin(angular_distance) * cos(lat_rad), ...
                                 cos(angular_distance) - sin(lat_rad) * sin(new_lat_rad));

    % Convert back to degrees
    new_lat = rad2deg(new_lat_rad);
    % Wrap longitude to the range [-180, 180] degrees
    new_lon = wrapTo180(rad2deg(new_lon_rad));
end

% -------------------------------------------------------------------------
% CALCULATE DISTANCE AND BEARING (Haversine & Spherical Bearing)
% -------------------------------------------------------------------------
function [distance_m, bearing_deg] = distance_bearing_local(params, lat1_deg, lon1_deg, lat2_deg, lon2_deg)
    % Calculates great-circle distance (using Haversine formula) and initial
    % bearing between two geographic points (using spherical formula).

    R_earth = params.a; % Use semi-major axis as approximate radius

    % Ensure inputs are numeric and convert degrees to radians
    lat1 = deg2rad(ensureNumeric_local(lat1_deg, 'lat1'));
    lon1 = deg2rad(ensureNumeric_local(lon1_deg, 'lon1'));
    lat2 = deg2rad(ensureNumeric_local(lat2_deg, 'lat2'));
    lon2 = deg2rad(ensureNumeric_local(lon2_deg, 'lon2'));

    % --- Haversine Distance ---
    dlat = lat2 - lat1;
    dlon = lon2 - lon1;
    a = sin(dlat/2)^2 + cos(lat1) * cos(lat2) * sin(dlon/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a)); % Angular distance in radians
    distance_m = R_earth * c; % Arc length

    % --- Initial Bearing ---
    % Formula for bearing from point 1 to point 2 on a sphere
    y = sin(dlon) * cos(lat2);
    x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    bearing_rad = atan2(y, x);
    % Convert to degrees and normalize to [0, 360) range from North
    bearing_deg = mod(rad2deg(bearing_rad), 360);
end

% -------------------------------------------------------------------------
% GEODETIC (LLH) TO ECEF CONVERSION
% -------------------------------------------------------------------------
function [X, Y, Z] = LLH_to_ECEF_local(params, lat_deg, lon_deg, h_m)
    % Converts Geodetic coordinates (Latitude, Longitude, Height above ellipsoid)
    % to Earth-Centered, Earth-Fixed (ECEF) Cartesian coordinates.

    lat = deg2rad(lat_deg); % Convert latitude to radians
    lon = deg2rad(lon_deg); % Convert longitude to radians

    % Calculate the radius of curvature in the prime vertical (N)
    N = params.a / sqrt(1 - params.e_sq * sin(lat)^2);

    % Calculate ECEF coordinates
    X = (N + h_m) * cos(lat) * cos(lon); % X points towards lat=0, lon=0
    Y = (N + h_m) * cos(lat) * sin(lon); % Y points towards lat=0, lon=90
    Z = (N * (1 - params.e_sq) + h_m) * sin(lat); % Z points towards North Pole
end

% -------------------------------------------------------------------------
% ECEF TO GEODETIC (LLH) CONVERSION (Iterative)
% -------------------------------------------------------------------------
function [lat_deg, lon_deg, h_m] = ECEF_to_LLH_local(params, X, Y, Z)
    % Converts ECEF (X, Y, Z) coordinates back to Geodetic LLH.
    % Uses an accurate iterative method (e.g., Bowring's or similar).

    % Longitude is calculated directly using atan2
    lon_rad = atan2(Y, X);
    lon_deg = wrapTo180(rad2deg(lon_rad)); % Range [-180, 180]

    % --- Iterative calculation for Latitude and Height ---
    p = sqrt(X^2 + Y^2); % Distance from the Z-axis

    % Handle singularity at the poles (p is very small)
    if p < 1e-9
        lat_deg = sign(Z) * 90;  % Latitude is +/- 90 degrees
        h_m     = abs(Z) - params.b; % Height above the pole (using semi-minor axis)
        return;
    end

    % Initial guess for latitude (spherical approximation) and height
    lat_rad = atan2(Z, p * (1 - params.e_sq)); % Initial guess for reduced latitude? Better start.
    h_m     = 0; % Initial guess for height

    % Iteratively refine latitude and height
    MAX_ITER  = 15;         % Maximum number of iterations
    TOLERANCE = 1e-6;      % millimeter precision - Convergence tolerance for height (meters)

    for i = 1:MAX_ITER
        lat_prev = lat_rad; % Store previous latitude for convergence check

        % Calculate radius of curvature in prime vertical (N) for current latitude guess
        N = params.a / sqrt(1 - params.e_sq * sin(lat_rad)^2);

        % Update height estimate
        h_m_new = p / cos(lat_rad) - N;

        % Update latitude estimate using the new height
        lat_rad = atan2(Z, p * (1 - params.e_sq * N / (N + h_m_new)));

        % Check for convergence (both height and latitude change significantly small)
        if abs(h_m_new - h_m) < TOLERANCE && abs(lat_rad - lat_prev) < (TOLERANCE / params.a) % Angle tolerance
             h_m = h_m_new; % Accept converged values
             break; % Exit loop
        end

        % Update values for next iteration
        h_m = h_m_new;

        % Check if max iterations reached without convergence
        if i == MAX_ITER
             warning('ECEF_to_LLH calculation did not fully converge within %d iterations.', MAX_ITER);
        end
    end

    % Convert final latitude to degrees
    lat_deg = rad2deg(lat_rad);
end

% -------------------------------------------------------------------------
% ECEF TO NED ROTATION MATRIX
% -------------------------------------------------------------------------
function R_ecef_to_ned = ECEF_to_NED_matrix_local(params, lat_deg, lon_deg)
    % Calculates the 3x3 rotation matrix to transform a vector from the
    % ECEF frame to the local NED frame at the specified geodetic point.
    % To use: V_ned = R_ecef_to_ned * V_ecef

    lat = deg2rad(lat_deg); % Convert reference point latitude to radians
    lon = deg2rad(lon_deg); % Convert reference point longitude to radians

    slat = sin(lat); clat = cos(lat);
    slon = sin(lon); clon = cos(lon);

    % The matrix rows represent the NED axes (North, East, Down)
    % expressed in terms of the ECEF axes (X, Y, Z).
    R_ecef_to_ned = [ -slat*clon, -slat*slon,  clat; ...  % North vector in ECEF
                      -slon,         clon,       0; ...  % East vector in ECEF
                      -clat*clon, -clat*slon, -slat ];   % Down vector in ECEF
end

% -------------------------------------------------------------------------
% NED TO ECEF ROTATION MATRIX
% -------------------------------------------------------------------------
function R_ned_to_ecef = NED_to_ECEF_matrix_local(params, lat_deg, lon_deg)
    % Calculates the 3x3 rotation matrix to transform a vector from the
    % local NED frame back to the ECEF frame.
    % This is simply the transpose of the ECEF_to_NED matrix.
    % To use: V_ecef = R_ned_to_ecef * V_ned

    R_ned_to_ecef = ECEF_to_NED_matrix_local(params, lat_deg, lon_deg)'; % Transpose
end

% -------------------------------------------------------------------------
% Helper: Ensure Numeric Scalar Input
% -------------------------------------------------------------------------
function num = ensureNumeric_local(val, varName)
    % Ensures input is a valid numeric scalar, converting from string if possible.
    if ischar(val) || isstring(val)
        num = str2double(val);
        if isnan(num) || ~isscalar(num)
            error('Input variable "%s" must be numeric or a valid numeric string scalar.', varName);
        end
    elseif ~isnumeric(val) || isempty(val) || ~isscalar(val)
         error('Input variable "%s" must be a non-empty, numeric scalar.', varName);
    else
        num = val; % Input is already a valid numeric scalar
    end
end