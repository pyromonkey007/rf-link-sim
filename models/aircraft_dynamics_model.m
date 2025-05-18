% =========================================================================
% AIRCRAFT DYNAMICS MODEL (FINAL - Corrected v5, TSPI Filtered)
% Version: FINAL 1.5
%
% Description:
%   Simulates the point-mass flight dynamics of the aircraft over time.
%   Manages the aircraft's state (position, attitude, velocity) and updates
%   it based on waypoint navigation, autopilot logic, environmental factors
%   (wind/gusts), and optional real-time user keyboard input.
%   It calculates Look/Depression angles to a configured reference site and
%   includes hooks/placeholders for potential real-time visualization updates.
%   Logs detailed Time-Space-Position Information (TSPI) at each time step,
%   filtering out any unused preallocated rows before output.
%
% Core Responsibilities:
%   - Initializes aircraft state from configuration.
%   - Iterates through simulation time steps (`dt`).
%   - Determines target state (heading, pitch, roll, speed) based on active
%     waypoint commands or keyboard override (`keyboard_input_controller.m`).
%   - Applies autopilot guidance logic (using simple P-controllers - gains in config).
%   - Updates aircraft attitude (pitch, roll, heading) based on calculated
%     rates and configured limits.
%   - Updates aircraft speed, converting between KCAS/TAS (`atmosphere` model)
%     and calculating Ground Speed considering wind/gusts (`config.environment`).
%   - Updates aircraft position (Lat, Lon, Alt) using the `earth` model and
%     ground track, including a basic terrain floor check using `terrain` model.
%   - Checks for waypoint arrival based on `config.flight.waypoint_radius_m`
%     and advances to the next waypoint.
%   - Calculates Look/Depression angles relative to `config.radar` site using
%     `compute_look_depression.m` (if enabled and configured).
%   - Estimates Time-To-Go (TTG) to the next waypoint based on ground speed.
%   - Calls real-time plotting update functions if enabled and configured
%     (includes rate limiting logic and placeholder calls for 3D rendering).
%   - Logs Time-Space-Position Information (TSPI), including Look/Dep angles,
%     to an array (`tspi_data`), filtering invalid rows post-loop. % <-- Updated
%   - Generates placeholder RF and Failure logs (`rf_log_cell`, `failure_log_cell`)
%     as RF calculations are performed post-simulation in `main_simulation.m`.
%   - Handles simulation termination conditions (max duration, end at last waypoint).
%   - Returns filtered logged data and simulation metadata. % <-- Updated
%
% Placeholders & Key Implementation Notes:
%   - **Autopilot Gains:** `config.flight.autopilot_*_gain` values require tuning.
%   - **Speed Control:** `config.flight.autopilot_speed_accel` provides basic acceleration limits.
%     Replace with a proper Thrust/Drag model for higher fidelity speed changes.
%   - **Real-time 3D Plots:** Calls to `realtime_3d_plot_init`, `..._update` are placeholders
%     (Patches 2, 3). Requires creating these functions and implementing 3D model loading
%     (STL/wireframe) and `patch` object updates based on aircraft state.
%   - **TAS <-> KCAS:** Requires `atmosphere.tas_to_kcas` function to be fully implemented
%     (including compressibility) for accurate KCAS display in real-time plot.
%   - **RF Calculation:** This model *does not* call `rf_propagation_model.compute_rf`.
%     RF results are calculated based on the logged TSPI *after* this simulation loop finishes,
%     handled within `main_simulation.m`. Placeholder logs are generated here.
%
% Key Fixes in this Version (1.5):
%   - **Added filtering of `tspi_log` after the simulation loop to remove unused,
%     zero-filled rows that caused plotting artifacts.** % <-- NEW FIX
%   - Corrected initialization and usage of `toc` for real-time plot update timer (`last_rt_plot_update_tic`).
%   - Removed erroneous reference to `simulation_metadata_placeholder` when defining TSPI headers.
%   - Fixed `MEXCEP` code analyzer issues by using correct `warning/error` format for MExceptions.
%   - Added checks for required model object validity.
%   - Improved comments and formatting.
%
% Maintainer: [Your Name/Team] 
% Last Updated: 2025-04-05 - Added TSPI filtering post-loop.
% =========================================================================
function [tspi_data, rf_log_cell, failure_log_cell, simulation_metadata] = aircraft_dynamics_model(config, earth, atmosphere, terrain, antennas)

    % --- Display Simulation Start ---
    fprintf('--- Starting Aircraft Dynamics Simulation ---\n');

    % --- Constants and Unit Conversions ---
    DEG2RAD = pi/180;         % Degrees to Radians
    RAD2DEG = 180/pi;         % Radians to Degrees
    FT2M    = 0.3048;         % Feet to Meters
    M2FT    = 1 / FT2M;       % Meters to Feet
    KTS2MPS = 0.514444;       % Knots to Meters per Second
    MPS2KTS = 1 / KTS2MPS;    % Meters per Second to Knots

    % --- Input Model Validation (Basic Checks) ---
    % Ensure required model objects with necessary functions were passed
    if ~isstruct(earth) || ~isfield(earth, 'LLH_to_ECEF') || ~isfield(earth, 'move') || ~isfield(earth, 'distance_bearing')
        error('DynamicsModel:InvalidInput', 'Aircraft Dynamics requires a valid earth model object with LLH_to_ECEF, move, and distance_bearing methods.');
    end
    if ~isstruct(atmosphere) || ~isfield(atmosphere, 'get_conditions') || ~isfield(atmosphere, 'kcas_to_tas') || ~isfield(atmosphere, 'tas_to_kcas')
        error('DynamicsModel:InvalidInput', 'Aircraft Dynamics requires a valid atmosphere model object with get_conditions, kcas_to_tas, and tas_to_kcas methods.');
    end
    if ~isstruct(terrain) || ~isfield(terrain, 'get_altitude')
         error('DynamicsModel:InvalidInput', 'Aircraft Dynamics requires a valid terrain model object with get_altitude method.');
    end
     % Antennas object is passed but not directly used in this version as RF is post-processed.

    % --- Look/Depression Angle Calculation Setup ---
    calc_look_dep = false; % Default: Don't calculate
    % Check if needed for real-time plot or potentially TSPI export
    lookdep_plot_enabled = isfield(config.visualization, 'ENABLE_REALTIME_LOOKDEP_PLOT') && config.visualization.ENABLE_REALTIME_LOOKDEP_PLOT;
    if lookdep_plot_enabled
        calc_look_dep = true; % Needed if plotting real-time
    end
    % Assume calculation is needed for TSPI export (can refine with explicit toggle)
    calc_look_dep = calc_look_dep || true;

    radar_coords = []; % Structure to hold radar site coordinates
    look_dep_enabled_flag = false; % Flag if calculation is possible and config is valid

    if calc_look_dep
        % Check if radar site is properly defined in configuration
        if ~isfield(config, 'radar') || ~isfield(config.radar, 'lat_deg') || isempty(config.radar.lat_deg) || ...
           ~isfield(config.radar, 'lon_deg') || isempty(config.radar.lon_deg) || ~isfield(config.radar, 'alt_m_msl')
            warning('DynamicsModel:ConfigWarning', ...
                    'Look/Depression angles requested or needed, but config.radar site (lat_deg, lon_deg, alt_m_msl) is incomplete. Skipping calculation.');
            calc_look_dep = false; % Disable calculation if config invalid
        else
            % Store radar coordinates for use in the loop
            radar_coords.lat = config.radar.lat_deg;
            radar_coords.lon = config.radar.lon_deg;
            radar_coords.alt = config.radar.alt_m_msl;
            if isfield(config.radar, 'site_id'), radar_coords.id = config.radar.site_id; else radar_coords.id = 'RADAR'; end
            fprintf('Look/Depression angles will be calculated relative to %s (%.4f, %.4f, %.1f m)\n', ...
                radar_coords.id, radar_coords.lat, radar_coords.lon, radar_coords.alt);
            look_dep_enabled_flag = true; % Calculation is configured and possible
        end
    end
    % Update calc_look_dep based on valid config check
    calc_look_dep = look_dep_enabled_flag;

    % --- Simulation Time Setup ---
    dt           = config.simulation.time_step_s;        % Simulation time step (s)
    max_duration = config.simulation.duration_s;      % Maximum simulation duration (s)
    num_steps_est = ceil(max_duration / dt) + 10;     % Estimated number of steps + buffer
    t_sim        = 0;                                  % Initialize current simulation time

    % --- Initial Aircraft State ---
    % Use a structure for state variables for clarity
    state = struct();
    state.time_s        = t_sim;
    state.lat_deg       = config.flight.initial_state.lat_deg;
    state.lon_deg       = config.flight.initial_state.lon_deg;
    state.alt_m_msl     = config.flight.initial_state.alt_ft_msl * FT2M; % Convert initial alt to meters
    state.heading_deg   = config.flight.initial_state.heading_deg;
    state.pitch_deg     = config.flight.initial_state.pitch_deg;
    state.roll_deg      = config.flight.initial_state.roll_deg;

    % Set initial speed based on first waypoint's target KCAS
    initial_kcas = 100; % Default if no waypoints
    if ~isempty(config.flight.waypoints), initial_kcas = config.flight.waypoints(1, 4); end
    try
        % Convert initial KCAS target to initial TAS using atmospheric model
        state.speed_mps_tas = atmosphere.kcas_to_tas(initial_kcas, state.alt_m_msl);
    catch ME_kcas
        warning(ME_kcas.identifier, 'Initial KCAS (%d kn) to TAS conversion failed: %s. Using 100 m/s.', initial_kcas, ME_kcas.message);
        state.speed_mps_tas = 100.0; % Assign a default value on error
    end

    % Initialize other state variables
    state.speed_mps_ground  = state.speed_mps_tas; % Initial ground speed approx = TAS (no wind factored in yet)
    state.vertical_speed_mps = state.speed_mps_tas * sind(state.pitch_deg); % Initial VS based on pitch
    state.roll_rate_dps      = 0.0;                % Initial roll rate (deg/s)
    state.pitch_rate_dps     = 0.0;                % Initial pitch rate (deg/s)
    state.heading_rate_dps   = 0.0;                % Initial heading rate (deg/s)

    % Initial velocity vector in North-East-Down (NED) frame [N; E; D] (m/s)
    state.velocity_ned_mps = [state.speed_mps_tas * cosd(state.heading_deg); ... % North component
                              state.speed_mps_tas * sind(state.heading_deg); ... % East component
                              -state.vertical_speed_mps];                       % Down component (opposite of VS)

    % --- Waypoint Management ---
    waypoints         = config.flight.waypoints;
    num_waypoints     = size(waypoints, 1);
    current_wp_index  = 1;          % Index of the current target waypoint
    target_wp         = [];         % Current target waypoint data row [Lat,Lon,AltFt,SpdKCAS,HdgCmd,PitchCmd,RollCmd]
    target_wp_alt_m   = 0;          % Target altitude in meters
    target_wp_speed_kcas = 0;       % Target speed in KCAS

    if num_waypoints >= current_wp_index
        target_wp          = waypoints(current_wp_index, :);
        target_wp_alt_m    = target_wp(3) * FT2M;  % Convert target alt to meters
        target_wp_speed_kcas = target_wp(4);
    else
        warning('DynamicsModel:ConfigWarning', 'No waypoints defined. Aircraft will fly straight or follow keyboard input.');
        current_wp_index = 0; % Indicate no active waypoint
    end

    waypoints_reached_count = 0;     % Counter for reached waypoints
    dist_to_wp_m          = Inf;   % Initialize distance to waypoint
    time_to_wp_s          = Inf;   % Initialize time-to-go to waypoint

    % --- Real-Time Input State Tracking ---
    realtime_active    = false;     % Flag if keyboard override is currently active
    user_pitch_adjust  = 0;         % Adjustment requested by user
    user_roll_adjust   = 0;         % Adjustment requested by user
    fig_handle_kbd     = [];        % Handle to figure used for keyboard capture

    if config.simulation.ENABLE_REALTIME_INPUT
        % Attempt to find an existing figure or create a hidden one for capture
        open_figs_kbd = findall(groot, 'Type', 'figure'); % Find all open figures
        if ~isempty(open_figs_kbd)
            fig_handle_kbd = open_figs_kbd(1); % Use the first one found
            try figure(fig_handle_kbd); % Bring focus to allow key capture
            catch, fig_handle_kbd=[]; end % Clear handle if figure invalid
            if ishandle(fig_handle_kbd), fprintf('Using Figure %d for keyboard input.\n', fig_handle_kbd.Number); end
        end
        % If no figure found or focus failed, try creating hidden figure
        if isempty(fig_handle_kbd)
            try
                fig_handle_kbd = figure('Visible', 'off', 'Name', 'KeyboardInputCapture'); % Invisible figure fallback
                fprintf('Created hidden figure for keyboard input.\n');
            catch ME_fig
                 warning(ME_fig.identifier, 'Could not find or create figure for keyboard input: %s. Keyboard input disabled.', ME_fig.message);
                 config.simulation.ENABLE_REALTIME_INPUT = false; % Disable locally if fails
            end
        end
    end

    % --- Real-time Plot Initialization ---
    rt_lookdep_handles = []; % Handles for Look/Depression Plot objects
    rt_3d_handles      = []; % Handles for 3D Plot objects (Placeholders)
    last_rt_plot_update_tic = tic; % Get initial time marker using tic
    pause(0.001);                % Short pause ensures first toc call is non-zero
    last_rt_plot_update_tic = tic; % Reset tic just before loop for precise start

    % Initialize Look/Dep Plot if enabled and configured
    if config.visualization.ENABLE_REALTIME_LOOKDEP_PLOT && calc_look_dep
        try
            % Initialize plot window and get graphics handles
            % Assumes function realtime_look_depression_init is on MATLAB path
            rt_lookdep_handles = realtime_look_depression_init(config);
            fprintf('Real-time Look/Depression Bin Fill Plot Initialized.\n');
        catch ME_rt_init
            warning(ME_rt_init.identifier, 'Failed to initialize real-time Look/Dep plot: %s. Disabling plot.', ME_rt_init.message);
            config.visualization.ENABLE_REALTIME_LOOKDEP_PLOT = false; % Disable locally
        end
    end

    % Initialize 3D Plot Placeholders if enabled
    if config.visualization.ENABLE_REALTIME_PLOTTING && ...
       (config.visualization.ENABLE_REALTIME_3D_AIRCRAFT || config.visualization.ENABLE_REALTIME_3D_FLIGHT_PATH)
        try
            fprintf('Initializing Real-time 3D plotting (Placeholders)...\n');
            % ** Placeholder Call: Replace with actual init function when implemented **
            % rt_3d_handles = realtime_3d_plot_init(config, terrain);

            % --- Minimal Placeholder Structure ---
            rt_3d_handles.fig = figure('Name', 'Realtime 3D View', 'NumberTitle', 'off');
            rt_3d_handles.ax = axes('Parent', rt_3d_handles.fig);
            hold(rt_3d_handles.ax, 'on'); grid(rt_3d_handles.ax, 'on'); view(3); axis equal;
            title(rt_3d_handles.ax, 'Real-time 3D View (Implementation Pending)');
            xlabel(rt_3d_handles.ax, 'Lon (deg)'); ylabel(rt_3d_handles.ax, 'Lat (deg)'); zlabel(rt_3d_handles.ax, 'Alt (ft)');
            rt_3d_handles.placeholder = true; % Mark as placeholder structure
            % --- End Placeholder ---
            fprintf('PLACEHOLDER: Real-time 3D Plot Initialized (Figure created, implementation pending).\n');
        catch ME_rt3d_init
             warning(ME_rt3d_init.identifier, 'Failed to initialize real-time 3D placeholder: %s. Disabling real-time 3D.', ME_rt3d_init.message);
             rt_3d_handles = []; % Ensure handles struct is empty on error
        end
    end

    % --- Logging Setup ---
    % Determine number of columns needed for TSPI log based on config
    num_tspi_cols_base = 12; % Base columns defined below
    num_tspi_cols = num_tspi_cols_base + (calc_look_dep * 2); % Add 2 if Look/Dep calculated

    % Preallocate logging arrays with a buffer for efficiency
    tspi_log         = zeros(num_steps_est, num_tspi_cols);
    % Note: RF logs are placeholders; actual RF calcs done post-simulation
    rf_log_cell      = cell(num_steps_est, 1);
    failure_log_cell = cell(num_steps_est, 1);

    % Define TSPI Headers dynamically
    base_headers = {'Time_s', 'Lat_deg', 'Lon_deg', 'Alt_m_MSL', ...
                    'Heading_deg', 'Pitch_deg', 'Roll_deg', ...
                    'TAS_mps', 'GroundSpeed_mps', 'VerticalSpeed_mps', ...
                    'Target_WP_Index', 'Keyboard_Active_Flag'};
    look_dep_headers = {};
    if calc_look_dep
        look_dep_headers = {'LookAngle_deg', 'DepressionAngle_deg'};
    end
    simulation_metadata = struct(); % Initialize metadata struct BEFORE adding fields
    simulation_metadata.tspi_headers = [base_headers, look_dep_headers]; % Store final headers

    log_index = 1; % Index for storing data in log arrays

    % --- Wind Setup ---
    wind_speed_mps        = config.environment.wind_speed_kts * KTS2MPS;
    % Convert wind FROM direction (deg CW from North) TO math angle (rad CCW from East)
    wind_dir_math_rad     = deg2rad(90 - config.environment.wind_direction_deg);
    wind_vector_north     = wind_speed_mps * sin(wind_dir_math_rad);
    wind_vector_east      = wind_speed_mps * cos(wind_dir_math_rad);
    current_gust_mps      = 0; % Initialize gust speed component
    last_gust_check_time  = -inf; % Ensures gust check happens on first applicable step
    gust_check_interval   = 1.0; % Check for new gusts every second (adjust if needed)

    fprintf('Wind: %.1f kts from %.0f deg (N:%.1f m/s, E:%.1f m/s)\n', ...
        config.environment.wind_speed_kts, config.environment.wind_direction_deg, wind_vector_north, wind_vector_east);

    % =====================================================================
    % --- Main Simulation Loop ---
    % =====================================================================
    fprintf('Starting simulation loop (dt=%.3f s, duration=%.1f s)...\n', dt, max_duration);
    loop_start_time       = tic;       % Timer for overall loop duration
    last_status_update_time = tic; % Use tic for consistent timing
    pause(0.001); last_status_update_time=tic; % Initialize console update timer
    simulation_metadata.completion_reason = 'Max duration reached'; % Default end reason

    while t_sim <= max_duration

        try % Wrap main loop body in try-catch for step-level error handling

            % --- 1. Get Current Atmospheric Conditions ---
            atmos_cond = atmosphere.get_conditions(state.alt_m_msl);
            if atmos_cond.density_kg_m3 <= 0 || isnan(atmos_cond.density_kg_m3)
                warning('DynamicsModel:AtmosError', 'Atmospheric density invalid (%.3f) at altitude %.1f m. Stopping simulation.', atmos_cond.density_kg_m3, state.alt_m_msl);
                simulation_metadata.completion_reason = 'Atmospheric Density Error';
                break; % Exit simulation loop
            end

            % --- 2. Handle Keyboard Input ---
            if config.simulation.ENABLE_REALTIME_INPUT
                [user_pitch_adjust, user_roll_adjust, realtime_active] = keyboard_input_controller(config, fig_handle_kbd);
            else
                realtime_active = false; user_pitch_adjust = 0; user_roll_adjust = 0; % Ensure inactive if disabled
            end

            % --- 3. Determine Target State (Autopilot vs Keyboard) ---
            % Initialize targets, get target speed from current WP
            target_heading_deg = state.heading_deg;
            target_pitch_deg = 0;
            target_roll_deg = 0;
            target_speed_kcas = 0;
            if current_wp_index > 0 && current_wp_index <= num_waypoints
                target_speed_kcas = target_wp(4);
            elseif ~isempty(config.flight.waypoints)
                 target_speed_kcas = config.flight.waypoints(end,4); % Use last WP speed if past end
            else
                 target_speed_kcas = state.speed_mps_tas * MPS2KTS; % Hold current speed if no WPs
            end

            % Apply Keyboard or Autopilot Logic
            if realtime_active
                % Keyboard Override: Directly adjust current attitude
                target_pitch_deg = state.pitch_deg + user_pitch_adjust;
                target_roll_deg = state.roll_deg + user_roll_adjust;
                % Maintain waypoint heading command if available and keyboard active
                if current_wp_index > 0 && current_wp_index <= num_waypoints
                    wp_hdg_cmd = target_wp(5);
                    if ~isnan(wp_hdg_cmd), target_heading_deg = wp_hdg_cmd; end
                end
            elseif current_wp_index > 0 && current_wp_index <= num_waypoints
                % Autopilot Logic: Navigate to current waypoint
                [dist_m, bearing_deg] = earth.distance_bearing(state.lat_deg, state.lon_deg, target_wp(1), target_wp(2));
                dist_to_wp_m = dist_m; % Update current distance

                % Get commands from waypoint definition
                wp_hdg_cmd = target_wp(5);
                wp_pitch_cmd = target_wp(6);
                wp_roll_cmd = target_wp(7);

                % --- Heading Control ---
                % Use waypoint heading command if defined and close, otherwise fly bearing
                if ~isnan(wp_hdg_cmd) && dist_m < (config.flight.waypoint_radius_m * 2)
                    target_heading_deg = wp_hdg_cmd;
                else
                    target_heading_deg = bearing_deg;
                end

                % --- Pitch Control ---
                % Use waypoint pitch command if defined, otherwise use simple altitude error feedback
                if ~isnan(wp_pitch_cmd)
                    target_pitch_deg = wp_pitch_cmd;
                else
                    alt_error_m = target_wp_alt_m - state.alt_m_msl;
                    % Simple Proportional controller for pitch based on altitude error
                    target_pitch_deg = max(min(alt_error_m * config.flight.autopilot_pitch_gain, 15), -15); % Limit pitch command
                end

                % --- Roll Control ---
                % Use waypoint roll command if defined, otherwise use simple heading error feedback
                if ~isnan(wp_roll_cmd)
                    target_roll_deg = wp_roll_cmd;
                else
                    heading_error_deg = mod(target_heading_deg - state.heading_deg + 180, 360) - 180; % Signed heading error (-180 to 180)
                    % Simple Proportional controller for roll based on heading error
                    target_roll_deg = max(min(heading_error_deg * config.flight.autopilot_roll_gain, 45), -45); % Limit roll command
                end
            else
                % No active waypoint (past last one or none defined): Hold current attitude and speed
                 target_heading_deg = state.heading_deg;
                 target_pitch_deg = state.pitch_deg;
                 target_roll_deg = state.roll_deg;
                 % target_speed_kcas already set to current speed equivalent earlier
            end

            % --- 4. Calculate Control Attitude Rates ---
            % Calculate errors between current and target attitudes
            pitch_error_deg = target_pitch_deg - state.pitch_deg;
            roll_error_deg = target_roll_deg - state.roll_deg;

            % Calculate commanded rates based on errors and gains
            pitch_rate_cmd_dps = pitch_error_deg * config.flight.autopilot_pitch_rate_gain;
            roll_rate_cmd_dps = roll_error_deg * config.flight.autopilot_roll_rate_gain;

            % Apply rate limits
            state.pitch_rate_dps = max(min(pitch_rate_cmd_dps, config.flight.max_pitch_rate_dps), -config.flight.max_pitch_rate_dps);
            state.roll_rate_dps = max(min(roll_rate_cmd_dps, config.flight.max_roll_rate_dps), -config.flight.max_roll_rate_dps);

            % Calculate heading rate (turn rate) based on roll angle and TAS
            g = 9.81; % Acceleration due to gravity (m/s^2)
            turn_rate_rad_s = 0;
            if abs(state.speed_mps_tas) > 1.0 % Avoid division by zero/small numbers
                turn_rate_rad_s = (g * tan(state.roll_deg * DEG2RAD)) / state.speed_mps_tas;
            end
            state.heading_rate_dps = turn_rate_rad_s * RAD2DEG;

            % --- 5. Update Attitude ---
            % Integrate rates over time step dt
            state.pitch_deg = state.pitch_deg + state.pitch_rate_dps * dt;
            state.roll_deg = state.roll_deg + state.roll_rate_dps * dt;
            state.heading_deg = mod(state.heading_deg + state.heading_rate_dps * dt, 360); % Keep heading 0-360

            % Apply pitch and roll limits
            state.pitch_deg = max(min(state.pitch_deg, 85), -85); % Limit pitch
            state.roll_deg = max(min(state.roll_deg, 80), -80); % Limit roll

            % --- 6. Update Speed ---
            % Convert target KCAS to target TAS
            try
                target_tas_mps = atmosphere.kcas_to_tas(target_speed_kcas, state.alt_m_msl);
            catch ME_tas
                 warning(ME_tas.identifier,'Target KCAS (%d kn) to TAS conversion failed: %s. Holding current TAS.', target_speed_kcas, ME_tas.message);
                 target_tas_mps = state.speed_mps_tas; % Hold current TAS on error
            end

            % Calculate speed error and apply acceleration limits
            speed_error_mps = target_tas_mps - state.speed_mps_tas;
            acceleration_mps2 = max(min(speed_error_mps / dt, config.flight.autopilot_speed_accel), -config.flight.autopilot_speed_accel);

            % Update TAS
            state.speed_mps_tas = max(state.speed_mps_tas + acceleration_mps2 * dt, 1.0); % Prevent TAS from going to zero or negative

            % --- Calculate Ground Speed considering Wind and Gusts ---
            effective_wind_north = wind_vector_north;
            effective_wind_east = wind_vector_east;
            % Apply Gusts if enabled
            if config.environment.enable_gusts
                if (t_sim - last_gust_check_time) >= gust_check_interval
                    gust_std_dev_mps = config.environment.gust_std_dev_kts * KTS2MPS;
                    % Simplified probability model: scales with std dev and interval
                    gust_prob = (gust_std_dev_mps / 5.0) * gust_check_interval;
                    if rand() < gust_prob
                        % Apply gust as a scaling factor on the base wind vector
                        gust_magnitude_mps = (rand() - 0.5) * 2 * gust_std_dev_mps; % Zero-mean gust component
                        gust_factor = (wind_speed_mps + gust_magnitude_mps) / max(wind_speed_mps, 1e-6); % Scale factor
                        effective_wind_north = wind_vector_north * gust_factor;
                        effective_wind_east = wind_vector_east * gust_factor;
                        % Optional: Add randomness to gust direction if needed
                    end
                    last_gust_check_time = t_sim; % Reset gust check timer
                end
            end

            % Calculate aircraft velocity components relative to air (TAS)
            tas_north_mps = state.speed_mps_tas * cosd(state.heading_deg);
            tas_east_mps = state.speed_mps_tas * sind(state.heading_deg);

            % Calculate ground velocity components (Air Vector + Wind Vector)
            gs_north_mps = tas_north_mps + effective_wind_north;
            gs_east_mps = tas_east_mps + effective_wind_east;

            % Calculate Ground Speed magnitude and Ground Track angle
            state.speed_mps_ground = sqrt(gs_north_mps^2 + gs_east_mps^2);
            ground_track_deg = mod(atan2d(gs_east_mps, gs_north_mps), 360); % atan2d handles quadrants correctly

            % --- 7. Update Position ---
            % Move aircraft based on ground speed and ground track
            dist_moved_m = state.speed_mps_ground * dt;
            [state.lat_deg, state.lon_deg] = earth.move(state.lat_deg, state.lon_deg, dist_moved_m, ground_track_deg);

            % Update altitude based on vertical speed
            state.vertical_speed_mps = state.speed_mps_tas * sind(state.pitch_deg); % VS based on TAS and pitch
            state.alt_m_msl = state.alt_m_msl + state.vertical_speed_mps * dt;

            % Apply Terrain Floor Check
            min_alt_agl_m = 10.0; % Minimum allowed height above ground level (m)
            min_alt_msl_m = 0.0; % Default minimum MSL altitude
            if terrain.data_loaded % Only check if terrain data is actually loaded
                try
                    terrain_alt_here_m = terrain.get_altitude(state.lat_deg, state.lon_deg);
                    min_alt_msl_m = terrain_alt_here_m + min_alt_agl_m;
                catch ME_alt
                    if log_index == 1 % Warn only once per simulation run
                         warning(ME_alt.identifier,'Terrain floor check failed: %s', ME_alt.message);
                    end
                    min_alt_msl_m = 0; % Fallback if terrain lookup fails
                end
            end
            state.alt_m_msl = max(state.alt_m_msl, min_alt_msl_m); % Ensure altitude is above floor

            % Update velocity vector in NED frame AFTER position/VS update
            state.velocity_ned_mps = [gs_north_mps; gs_east_mps; -state.vertical_speed_mps]; % [North; East; Down]

            % --- 8. Check Waypoint Arrival & Calc TTG ---
            if current_wp_index > 0 && current_wp_index <= num_waypoints
                % Recalculate distance to the current target waypoint
                [dist_to_wp_m, ~] = earth.distance_bearing(state.lat_deg, state.lon_deg, target_wp(1), target_wp(2));

                % Calculate Time-To-Go (TTG)
                if state.speed_mps_ground > 0.1
                    time_to_wp_s = dist_to_wp_m / state.speed_mps_ground;
                else
                    time_to_wp_s = Inf; % Avoid division by zero if stationary
                end

                % Check if waypoint arrival radius is reached
                if dist_to_wp_m < config.flight.waypoint_radius_m
                    fprintf('--- WP %d Reached (T=%.1fs) ---\n', current_wp_index, t_sim);
                    waypoints_reached_count = waypoints_reached_count + 1;
                    current_wp_index = current_wp_index + 1; % Advance to next waypoint index

                    % Check if there are more waypoints
                    if current_wp_index <= num_waypoints
                        % Load next waypoint data
                        target_wp = waypoints(current_wp_index, :);
                        target_wp_alt_m = target_wp(3) * FT2M;
                        target_wp_speed_kcas = target_wp(4);
                        fprintf('Proceeding to WP %d\n', current_wp_index);
                        % Update distance/TTG to the new waypoint immediately
                        [dist_to_wp_m, ~] = earth.distance_bearing(state.lat_deg, state.lon_deg, target_wp(1), target_wp(2));
                        if state.speed_mps_ground > 0.1, time_to_wp_s = dist_to_wp_m / state.speed_mps_ground; else time_to_wp_s = Inf; end
                    elseif config.simulation.END_AT_LAST_WAYPOINT
                        % If configured to end after last waypoint, break the loop
                        simulation_metadata.completion_reason = 'Final waypoint reached';
                        break; % Exit simulation loop
                    else
                        % Past last waypoint, but not configured to stop
                        fprintf('--- Final WP Reached (T=%.1fs). Continuing run. ---\n', t_sim);
                        dist_to_wp_m = 0; time_to_wp_s = 0;
                        current_wp_index = num_waypoints + 1; % Indicate past last WP
                    end
                end
            else % Past last WP or no WPs defined
                dist_to_wp_m = NaN; % No meaningful distance
                time_to_wp_s = NaN; % No meaningful TTG
                if current_wp_index == 0, current_wp_index=NaN; end % Indicate no WP defined if started with none
            end

            % --- 9. Compute Look/Depression Angles ---
            look_angle_deg = NaN; depression_angle_deg = NaN; % Initialize as NaN
            if calc_look_dep % Only calculate if enabled and configured correctly
                try
                    [look_angle_deg, depression_angle_deg] = compute_look_depression_user(...
                        radar_coords.lat, radar_coords.lon, radar_coords.alt, ...
                        state.lat_deg, state.lon_deg, state.alt_m_msl, ...
                        state.roll_deg, state.pitch_deg, state.heading_deg, earth);
                catch ME_ld
                    if log_index == 1 % Warn only once
                         warning(ME_ld.identifier,'Look/Dep calc failed: %s', ME_ld.message);
                    end
                    % Keep angles as NaN on error
                end
            end

            % --- 10. RF Calculations (Placeholder Log Entry) ---
            % Actual RF calculations are performed in main_simulation.m after this loop
            rf_log_cell{log_index} = struct('status', 'Not Calculated in Dynamics');
            failure_log_cell{log_index} = struct('reason', 'Not Calculated in Dynamics');

            % --- 11. Log Current State (TSPI) ---
            % Check if log array needs to grow (should be rare with proper estimation)
            if log_index > size(tspi_log, 1)
                warning('DynamicsModel:LogGrow','Growing TSPI log at step %d', log_index);
                tspi_log = [tspi_log; zeros(num_steps_est, num_tspi_cols)]; % Append more rows
            end
            % Assemble the base TSPI data for this time step
            base_tspi_data = [t_sim, state.lat_deg, state.lon_deg, state.alt_m_msl, ...
                              state.heading_deg, state.pitch_deg, state.roll_deg, ...
                              state.speed_mps_tas, state.speed_mps_ground, state.vertical_speed_mps, ...
                              current_wp_index, double(realtime_active)];
            % Assemble Look/Depression angle data if calculated
            look_dep_log_data = [];
            if calc_look_dep
                look_dep_log_data = [look_angle_deg, depression_angle_deg];
            end
            % Combine and store in the preallocated log matrix
            tspi_log(log_index, :) = [base_tspi_data, look_dep_log_data];

            % --- 12. Real-time Visualization Updates ---
            % Rate limit the plot updates to avoid excessive slowdown
            plot_update_interval_s = 1 / config.visualization.REALTIME_LOOKDEP_UPDATE_RATE_HZ; % Use Look/Dep rate for now
            do_plot_update = false; % Flag to indicate if plots should update this step

            % Check timer ONLY if enough time has passed since last update
            if toc(last_rt_plot_update_tic) >= plot_update_interval_s
                 do_plot_update = true;
                 % Timer reset moved inside the 'if do_plot_update' block below
            end

            % --- Perform updates ONLY if do_plot_update is true ---
            if do_plot_update
                % --- Update Look/Dep Bin Fill Plot ---
                if config.visualization.ENABLE_REALTIME_LOOKDEP_PLOT && ~isempty(rt_lookdep_handles) && calc_look_dep
                    % Check if data collection is active based on waypoint range
                    collect_data = (current_wp_index >= config.visualization.REALTIME_LOOKDEP_START_WP && ...
                                    current_wp_index <= config.visualization.REALTIME_LOOKDEP_END_WP);
                    % Only update if collecting and angles are valid
                    if collect_data && isfinite(look_angle_deg) && isfinite(depression_angle_deg)
                        try % Wrap update call in try-catch
                            % Prepare data structure for the update function
                            current_disp_data = struct();
                            current_disp_data.alt_ft = state.alt_m_msl * M2FT;
                            try % Convert TAS to KCAS for display (can fail)
                                current_disp_data.kcas = atmosphere.tas_to_kcas(state.speed_mps_tas, state.alt_m_msl);
                            catch
                                current_disp_data.kcas = NaN;
                            end
                            current_disp_data.hdg = state.heading_deg;
                            current_disp_data.pitch = state.pitch_deg;
                            current_disp_data.roll = state.roll_deg;
                            current_disp_data.wp = current_wp_index;
                            current_disp_data.ttg = time_to_wp_s;

                            % Call the update function, assuming it exists on path
                            realtime_look_depression_update(rt_lookdep_handles, look_angle_deg, depression_angle_deg, current_disp_data);
                        catch ME_rt_upd
                             warning(ME_rt_upd.identifier, 'Failed RT Look/Dep update: %s', ME_rt_upd.message);
                        end
                    end
                end

                % --- Update Real-time 3D Plot (Placeholders) ---
                if ~isempty(rt_3d_handles) && isfield(rt_3d_handles,'placeholder') % Check if placeholder exists
                    try % Placeholder update logic
                        if isfield(rt_3d_handles, 'ax') && ishandle(rt_3d_handles.ax) % Check if axes still exist
                            % Example: Update simple marker position for aircraft
                            if ~isfield(rt_3d_handles, 'ac_marker') || ~ishandle(rt_3d_handles.ac_marker)
                                % Create marker if it doesn't exist
                                rt_3d_handles.ac_marker = plot3(rt_3d_handles.ax, state.lon_deg, state.lat_deg, state.alt_m_msl*M2FT, ...
                                                                'rp','MarkerSize',10,'MarkerFaceColor','r');
                            else
                                % Update existing marker position
                                set(rt_3d_handles.ac_marker, 'XData', state.lon_deg, 'YData', state.lat_deg, 'ZData', state.alt_m_msl*M2FT);
                            end
                            % ** Replace above with calls to actual update functions when implemented **
                            % E.g., update 3D model orientation, flight path line, etc.
                        end
                        drawnow limitrate; % Refresh the 3D plot window without overwhelming system
                    catch ME_rt3d
                         warning(ME_rt3d.identifier, 'Failed RT 3D update placeholder: %s', ME_rt3d.message);
                         rt_3d_handles = []; % Clear handles to prevent further attempts on error
                    end
                end % End 3D Plot update check

                % --- Reset Timer AFTER updates are done for this interval ---
                last_rt_plot_update_tic = tic; % Get new start time marker for next interval

            end % End if do_plot_update

            % --- 13. Advance Time ---
            t_sim     = t_sim + dt;       % Increment simulation time
            log_index = log_index + 1;    % Increment log storage index

            % --- 14. Console Status Update (Rate Limited) ---
            if (toc(last_status_update_time) >= 5.0) || log_index == 2 % Update every 5s or on second step
                fprintf('T+%.1fs | Alt:%.0fft | Hdg:%.1f | TAS:%.0fkn | GS:%.0fkn | Pitch:%.1f | Roll:%.1f | WP:%d | TTG:%.1fs\n', ...
                        t_sim, state.alt_m_msl*M2FT, state.heading_deg, state.speed_mps_tas*MPS2KTS, ...
                        state.speed_mps_ground*MPS2KTS, state.pitch_deg, state.roll_deg, current_wp_index, time_to_wp_s);
                last_status_update_time = tic; % Reset console update timer
            end

            % --- 15. Check Termination Condition (Max Duration) ---
            if t_sim >= max_duration % Use >= for robustness against time step precision issues
                % simulation_metadata.completion_reason = 'Max duration reached'; % Already set by default
                break; % Exit simulation loop
            end
            % Loop termination via END_AT_LAST_WAYPOINT handled in waypoint arrival check (Step 8)

        catch ME_step % Catch errors occurring within a single simulation step
            warning(ME_step.identifier, ...
                    'Error during step %d (T=%.2f s): %s\nStack:\n%s\nAttempting to continue...', ...
                    log_index, t_sim, ME_step.message, getReport(ME_step, 'basic'));
            % Log the error details
            failure_log_cell{log_index} = sprintf('Step Error @ T=%.2f: %s (%s)', t_sim, ME_step.message, ME_step.identifier);
            % Skip to next step (or break if preferred for critical errors)
            t_sim = t_sim + dt; log_index = log_index + 1; % Ensure time advances if continuing
            if log_index > num_steps_est + 10 % Add safety break if error repeats excessively
                simulation_metadata.completion_reason = 'Repeated Step Error Limit Reached';
                warning('DynamicsModel:ErrorLimit','Exiting loop due to repeated step errors.');
                break;
            end
        end % End try-catch for main loop body

    end % End of main simulation loop (while t_sim <= max_duration)

    % =====================================================================
    % --- Post-Loop Processing & Cleanup ---
    % =====================================================================
    loop_duration = toc(loop_start_time); % Calculate total loop execution time
    fprintf('--- Simulation Loop Finished (Actual CPU Time: %.2f seconds) ---\n', loop_duration);

    % --- ** FIX: Remove unused preallocated rows from tspi_log ** ---
    % Find the rows where actual data was logged. We assume the simulation always
    % logs time > 0 for valid steps. Preallocated rows will have time == 0.
    valid_log_indices = tspi_log(1:log_index-1, 1) > 0; % Check time column (col 1) up to the last attempted index

    % Keep only the rows corresponding to valid indices
    tspi_data_filtered = tspi_log(valid_log_indices, :);

    % Also filter the corresponding cell arrays for RF and Failures
    if ~isempty(rf_log_cell)
        rf_log_cell_filtered = rf_log_cell(valid_log_indices);
    else
        rf_log_cell_filtered = {};
    end
     if ~isempty(failure_log_cell)
        failure_log_cell_filtered = failure_log_cell(valid_log_indices);
    else
        failure_log_cell_filtered = {};
    end
    % --- End of FIX ---

    % --- Assign Filtered Data to Output Variables ---
    actual_steps = size(tspi_data_filtered, 1); % Get count from filtered data
    if actual_steps < 1
        warning('DynamicsModel:NoSteps', 'Simulation completed with zero valid logged steps after filtering.');
        tspi_data        = []; % Ensure empty output if no steps run/logged correctly
        rf_log_cell      = {};
        failure_log_cell = {};
    else
        % Assign the filtered data to the output variables
        tspi_data        = tspi_data_filtered;
        rf_log_cell      = rf_log_cell_filtered;
        failure_log_cell = failure_log_cell_filtered;
    end

    % --- Package Simulation Metadata ---
    if ~isfield(simulation_metadata,'completion_reason'), simulation_metadata.completion_reason = 'Ended Unexpectedly'; end % Ensure reason exists
    simulation_metadata.total_sim_time_s = max(0, t_sim - dt); % Use actual end time achieved (or 0 if no steps)
    simulation_metadata.time_step_s      = dt;
    simulation_metadata.num_steps        = actual_steps; % Use count AFTER filtering
    simulation_metadata.waypoints_reached= waypoints_reached_count;
    simulation_metadata.final_state      = state; % Store the last valid aircraft state
    % simulation_metadata.tspi_headers was already stored during setup

    fprintf('Simulation completed. Reason: %s\n', simulation_metadata.completion_reason);
    fprintf('Total valid steps logged: %d, Total simulated time: %.2f s\n', simulation_metadata.num_steps, simulation_metadata.total_sim_time_s);

    % --- Cleanup Real-time Plot Figures ---
    % Leave them open by default for user inspection after simulation ends
    if ~isempty(rt_lookdep_handles) && isfield(rt_lookdep_handles,'fig') && ishandle(rt_lookdep_handles.fig)
        fprintf('Real-time Look/Depression plot window remains open.\n');
    end
    if ~isempty(rt_3d_handles) && isfield(rt_3d_handles,'fig') && ishandle(rt_3d_handles.fig)
        fprintf('Real-time 3D plot window remains open.\n');
    end

    % --- Assign outputs for function return ---
    % Outputs tspi_data, rf_log_cell, failure_log_cell were assigned above after filtering

end % END OF FUNCTION aircraft_dynamics_model