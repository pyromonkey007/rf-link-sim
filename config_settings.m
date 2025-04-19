% =========================================================================
% CONFIGURATION SETTINGS for High-Fidelity Aircraft RF Link Simulation
% Version: FINAL 1.1 (Re-Commented v2)
%
% Description:
%   This is the CENTRAL configuration file for the entire simulation. All
%   user-adjustable parameters, simulation controls, file paths, model choices,
%   and feature toggles should be set here. The script generates a 'config'
%   structure which is then passed to other modules.
%
% How to Use:
%   1. **File Paths:** CRITICAL - Update DEM_FILE_PATH, STL_FILE,
%      WIREFRAME_FILE, and antenna_pattern_file paths below to match the
%      location and names of YOUR data files within the 'data/' directory.
%   2. **Scenario:** Define the flight plan by setting the initial_state
%      and waypoints array.
%   3. **RF Setup:** Define aircraft transmitters and ground station receivers,
%      including their frequencies, powers, gains, sensitivities, bandwidths,
%      and supported transmitter IDs (`supported_tx_ids`).
%   4. **Toggles:** Enable or disable major features (RF, Terrain, Masking,
%      Visualizations, Real-time input) using the boolean `ENABLE_...` flags.
%   5. **Model Choices:** Select atmospheric model, masking method, etc.
%   6. **Tuning:** Adjust parameters like time step, autopilot gains, rate
%      limits, plot update rates for desired performance and fidelity.
%   7. **Save:** Save this file before running `main_simulation.m`.
%
% Structure:
%   The configuration is organized into sub-structures (e.g., config.simulation,
%   config.flight, config.rf) for better readability and access.
%
% Units:
%   - Default: SI units (meters, seconds, Hz, Kelvin, etc.) unless otherwise specified.
%   - Exceptions Noted:
%       - Waypoint Altitude input in Feet (alt_ft_msl) - converted to meters internally.
%       - Waypoint Speed input in Knots Calibrated Air Speed (KCAS) - converted to TAS (m/s) internally.
%       - Wind Speed input in Knots (kts) - converted to m/s internally.
%       - Angles: Degrees (deg) throughout the configuration (converted to radians internally where needed).
%       - Power: dBW (database Watts) for Tx power, dBm (database milliwatts) for Rx sensitivity.
%       - Gain: dBi (database isotropic).
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-05] - Re-commented, added validation helper.
% =========================================================================
function config = config_settings()
    % Using a function ensures clean workspace and allows returning the config struct.

    % Clear previous config structure from workspace (optional, good practice)
    clear config;

    % Initialize the main configuration structure
    config = struct();

    % =====================================================================
    % 1. Simulation Control & Timing
    %    Defines overall simulation run parameters.
    % =====================================================================
    config.simulation = struct();

    % Maximum duration the simulation will run for.
    config.simulation.duration_s = 60 %60*60*2; % seconds (e.g., 1 hour)

    % Simulation time step. This is the fundamental interval at which the
    % aircraft state is updated. Smaller values increase fidelity (smoother
    % motion, more frequent RF checks) but significantly increase computation time.
    % Values between 0.01s and 0.1s are common trade-offs.
    config.simulation.time_step_s = .5; % seconds

    % Master toggle for allowing real-time keyboard control override.
    % If true, pressing control keys (Arrows/WASD) during simulation can
    % temporarily override autopilot commands. Requires an active figure window.
    config.simulation.ENABLE_REALTIME_INPUT = true;

    % If real-time input is enabled, this is the duration (in seconds)
    % without any control key presses before the simulation automatically
    % reverts to following the waypoint autopilot commands.
    config.simulation.realtime_input_timeout_s = 5.0; % seconds

    % If true, the simulation will terminate automatically as soon as the
    % aircraft reaches the final defined waypoint in `config.flight.waypoints`,
    % even if `config.simulation.duration_s` has not elapsed. If false,
    % the simulation runs for the full duration.
    config.simulation.END_AT_LAST_WAYPOINT = true;

    % =====================================================================
    % 2. Output Settings
    %    Controls how and where simulation results are saved.
    % =====================================================================
    config.output = struct();

    % The base directory where all simulation output folders will be created.
    % It's recommended to keep this as 'output'.
    config.output.BASE_FOLDER = 'output';

    % If true, a unique subfolder is created within BASE_FOLDER for each simulation run,
    % named with the date and time (e.g., 'output/sim_YYYYMMDD_HHMMSS'). This prevents
    % overwriting results from previous runs. Highly recommended.
    % If false, outputs go directly into BASE_FOLDER (overwriting previous results).
    config.output.ENABLE_TIMESTAMP_FOLDER = true;

    % The actual output folder path for the current run.
    % ** Leave this empty.** It will be automatically generated based on the
    % BASE_FOLDER and ENABLE_TIMESTAMP_FOLDER settings when this script runs.
    config.output.FOLDER = '';

    % --- Data Export Toggles ---

    % If true, save generated plots (from visualization_model, ray_trace, etc.)
    % as image files (typically PNG) in the 'figures/' subfolder within the run's output folder.
    % Controlled by `snapshot_generator.m`. (Patch 7)
    config.output.ENABLE_SNAPSHOT_EXPORT = true;

    % If true, save the primary Time-Space-Position Information (TSPI) data matrix.
    % Controlled by `export_TSPI_matrix.m`.
    config.output.ENABLE_TSPI_EXPORT = true;

    % Format for the main TSPI output file ('csv' or 'mat').
    % CSV is human-readable, MAT is generally better for reloading into MATLAB.
    config.output.TSPI_FORMAT = 'csv';

    % If true, save a detailed MATLAB .mat file containing the config structure,
    % raw cell array logs (rf_log_cell, failure_log_cell), the processed RF data
    % structure, the full TSPI data matrix, and simulation metadata. Saved in the
    % 'logs/' subfolder. Very useful for detailed post-run analysis and debugging. (Patch 6)
    config.output.ENABLE_FULL_RAW_DATA_LOG = true;

    % Format for the detailed raw log file ('mat' is strongly recommended).
    config.output.RAW_DATA_FORMAT = 'mat';

    % If true, generate the HTML summary dashboard (`mission_summary.html`) at the
    % end of the run using `generate_html_dashboard.m`. (Patch 17)
    % Requires static assets (CSS/JS) in the 'dashboard/' project folder.
    config.output.ENABLE_HTML_DASHBOARD = true;

    % --- Auto-generate Output Folder Name ---
    % This logic constructs the full output path based on the settings above.
    if config.output.ENABLE_TIMESTAMP_FOLDER
        timestamp = datestr(now, 'yyyymmdd_HHMMSS'); % Generate timestamp string
        config.output.FOLDER = fullfile(config.output.BASE_FOLDER, ['sim_' timestamp]);
    else
        config.output.FOLDER = config.output.BASE_FOLDER; % Use base folder directly
    end

    % --- Create Output Directories ---
    % Ensure the necessary output folders exist before the simulation starts writing files.
    % Uses try-catch blocks for robustness if folder creation fails (e.g., permissions).
    if ~isfolder(config.output.FOLDER)
        try mkdir(config.output.FOLDER);
        catch ME_mkdir
            error('Failed to create main output folder: %s\nError: %s', config.output.FOLDER, ME_mkdir.message);
        end
    end
    % Create standard subfolders for organization
    subfolders = {'figures', 'logs', 'dashboard'}; % Add others if needed
    for i = 1:length(subfolders)
        subfolder_path = fullfile(config.output.FOLDER, subfolders{i});
        if ~isfolder(subfolder_path)
            try mkdir(subfolder_path);
            catch ME_mkdir_sub
                % Warn but don't necessarily stop if a subfolder fails
                warning('Failed to create output subfolder: %s\nError: %s', subfolder_path, ME_mkdir_sub.message);
            end
        end
    end

    % =====================================================================
    % 3. Flight Dynamics & Waypoints
    %    Defines the aircraft's initial state, flight path, and control limits.
    % =====================================================================
    config.flight = struct();

    % --- Initial Aircraft State ---
    % Defines where the aircraft starts at T=0.
    % config.flight.initial_state = struct( ...
    %     'lat_deg', 34.4208, ...    % Latitude (degrees) - e.g., Santa Clarita approx
    %     'lon_deg', -118.5394, ... % Longitude (degrees) - e.g., Santa Clarita approx
    %     'alt_ft_msl', 2000, ...   % Altitude (feet above Mean Sea Level)
    %     'heading_deg', 90, ...    % Initial Heading (degrees, 0=North, 90=East, 180=S, 270=W)
    %     'pitch_deg', 0, ...       % Initial Pitch angle (degrees, +ve = nose up)
    %     'roll_deg', 0 ...         % Initial Roll angle (degrees, +ve = right wing down)
    % );
    config.flight.initial_state = struct( ...
        'lat_deg', 35.3, ...    % Latitude (degrees) - e.g., Santa Clarita approx
        'lon_deg', -118, ... % Longitude (degrees) - e.g., Santa Clarita approx
        'alt_ft_msl', 2000, ...   % Altitude (feet above Mean Sea Level)
        'heading_deg', 90, ...    % Initial Heading (degrees, 0=North, 90=East, 180=S, 270=W)
        'pitch_deg', 0, ...       % Initial Pitch angle (degrees, +ve = nose up)
        'roll_deg', 0 ...         % Initial Roll angle (degrees, +ve = right wing down)
    );
    % --- Waypoints ---
    % Defines the flight path as a sequence of points.
    % Format: Each row is a waypoint:
    % [Latitude(deg), Longitude(deg), Altitude(ft MSL), Speed(KCAS), TargetHeading(deg), PitchCommand(deg), RollCommand(deg)]
    % - TargetHeading: Desired heading *at* the waypoint. If NaN, autopilot flies direct bearing or maintains current heading near WP.
    % - PitchCommand: Specific pitch angle command *during transit* to this WP. If NaN, autopilot uses altitude guidance logic.
    % - RollCommand: Specific roll angle command *during transit* to this WP. If NaN, autopilot uses heading guidance logic.
    % config.flight.waypoints = [ ...
    %     % Lat,   Lon,      Alt(ft), Spd(KCAS), HdgCmd, PitchCmd, RollCmd
    %     34.45, -118.45,  5000,    250,          90,     NaN,      NaN;  % WP 1: Fly East
    %     34.50, -118.30, 10000,    300,          45,       5,       15;  % WP 2: Climb (+5deg), Turn Right (+15deg roll), Target Hdg 45
    %     34.45, -118.15, 10000,    300,         NaN,     NaN,      -15;  % WP 3: Hold Alt, Turn Left (-15deg roll), Fly bearing to WP4
    %     34.35, -118.15,  8000,    280,         180,      -5,      NaN;  % WP 4: Descend (-5deg), Target Hdg South (180)
    %     % Add more waypoints as needed for your scenario...
    % ];
    config.flight.waypoints = [ ...
        % Lat,   Lon,      Alt(ft), Spd(KCAS), HdgCmd, PitchCmd, RollCmd
        35.64, -117.35,  15000,    250,          20,     NaN,      NaN;  % WP 1: Fly East
        36.5739, -117.134, 15000,    250,          45,       NaN,       NaN;  % WP 2: Climb (+5deg), Turn Right (+15deg roll), Target Hdg 45
        37.6556, -115.398, 15000,    250,         90,     NaN,      NaN;  % WP 3: Hold Alt, Turn Left (-15deg roll), Fly bearing to WP4
        % Add more waypoints as needed for your scenario...
    ];
    % Arrival Radius: Distance (meters) from a waypoint's geographic coordinates
    % within which the aircraft is considered to have "reached" the waypoint.
    config.flight.waypoint_radius_m = 500; % meters

    % --- Flight Control Performance Limits ---
    % Maximum rate at which the aircraft can change its roll angle.
    config.flight.max_roll_rate_dps = 30;  % degrees/second
    % Maximum rate at which the aircraft can change its pitch angle.
    config.flight.max_pitch_rate_dps = 5;  % degrees/second
    % Maximum positive G-load constraint (Not currently enforced in simple dynamics model).
    config.flight.max_g_load = 3.0;
    % Maximum negative G-load constraint (Not currently enforced).
    config.flight.min_g_load = -1.0;

    % --- Autopilot Tuning Parameters ---
    % ** THESE GAINS TYPICALLY REQUIRE TUNING based on aircraft type & desired response! **
    % Proportional gain mapping altitude error (meters) to target pitch angle (degrees).
    % Higher value -> more aggressive altitude changes.
    config.flight.autopilot_pitch_gain = 0.1;
    % Proportional gain mapping heading error (degrees) to target roll angle (degrees).
    % Higher value -> more aggressive turns.
    config.flight.autopilot_roll_gain = 1.0;
    % Proportional gain mapping pitch angle error (degrees) to pitch rate command (degrees/second).
    % Higher value -> faster pitch response towards target pitch.
    config.flight.autopilot_pitch_rate_gain = 1.0;
    % Proportional gain mapping roll angle error (degrees) to roll rate command (degrees/second).
    % Higher value -> faster roll response towards target roll.
    config.flight.autopilot_roll_rate_gain = 2.0;
    % Simple maximum acceleration/deceleration used when changing speed (m/s^2).
    % ** PLACEHOLDER: Needs replacement with a realistic Thrust/Drag model. **
    config.flight.autopilot_speed_accel = 5.0;

    % =====================================================================
    % 4. Environmental Settings
    %    Defines atmospheric conditions and wind.
    % =====================================================================
    config.environment = struct();

    % Select atmospheric profile for density/temperature/pressure calculations.
    % Affects TAS calculation from KCAS and potentially atmospheric RF loss.
    % Options: 'standard', 'hot', 'cold', 'humid', 'conservative' (defines T0, P0, Lapse Rate).
    config.environment.MODEL = 'standard';

    % --- Wind Conditions ---
    % Average steady wind speed.
    config.environment.wind_speed_kts = 10;   % knots
    % Direction the wind is COMING FROM (meteorological convention).
    % 0=North, 90=East, 180=South, 270=West.
    config.environment.wind_direction_deg = 270; % Wind from the West
    % Enable random wind gusts (superimposed on steady wind).
    config.environment.enable_gusts = true;
    % Standard deviation of wind gusts (knots). Used in simple random gust generation probability.
    config.environment.gust_std_dev_kts = 5;

    % =====================================================================
    % 5. Earth Model
    %    Specifies the model used for geodetic calculations.
    % =====================================================================
    config.earth = struct();
    % Earth model for geodetic calculations ('wgs84' recommended).
    % 'sphere' could be added as an option for simpler (less accurate) calcs.
    config.earth.model = 'wgs84';

    % =====================================================================
    % 6. Terrain Settings
    %    Configures terrain data loading and usage.
    % =====================================================================
    config.terrain = struct();

    % Master toggle for enabling terrain elevation lookups and Line-of-Sight (LOS) checks.
    % If false, simulation assumes a flat Earth at sea level for LOS.
    config.terrain.ENABLE_TERRAIN_LOS = true;

    % ** IMPORTANT: Set this path to your Digital Elevation Model (DEM) file. **
    % Can be GeoTIFF (.tif, .dt2, etc.) or CSV (.csv with Lat,Lon,Elev columns).
    config.terrain.DEM_FILE_PATH = 'C:\Users\pyrom\OneDrive\Desktop\Terrain Data MASTER\merged_dem.tif'; % <<< SET YOUR FILE PATH HERE

    % If true and Mapping Toolbox is available, prioritize using `readgeoraster` (handles various GeoTIFFs/formats).
    % If false or toolbox unavailable, will attempt basic CSV fallback loader.
    config.terrain.USE_GEOTIFF = true;

    % Downsampling factor for DEM grid. 1 = no downsampling (full resolution).
    % Values > 1 (e.g., 2, 4, 10) reduce resolution by skipping points, which
    % speeds up processing and reduces memory usage but decreases terrain detail.
    config.terrain.DOWNSAMPLE_FACTOR = 1;

    % Interpolation method used by `terrain.get_altitude` for lookups between DEM grid points.
    % Options match `interp2`: 'nearest', 'linear', 'bilinear' (same as linear for 2D), 'cubic', 'spline'.
    % 'bilinear' (or 'linear') is usually a good balance of speed and accuracy.
    config.terrain.INTERPOLATION_METHOD = 'linear';

    % =====================================================================
    % 7. RF Propagation Settings
    %    Configures transmitters, receivers, and physics models for RF links.
    % =====================================================================
    config.rf = struct();

    % Master toggle for enabling all RF link budget calculations and logging.
    % If false, RF sections are skipped entirely.
    config.rf.ENABLE_RF_MODULE = true;

    % --- Transmitters (On Aircraft) ---
    % Cell array, where each cell defines one transmitter using a structure.
    config.rf.transmitters = { ...
        % 'id': Unique string identifier. ** Must match entries in receiver 'supported_tx_ids'. ** No spaces recommended.
        % 'freq_hz': Center frequency (Hz). Used for FSPL, Doppler, Atmos Loss.
        % 'power_dbw': Transmit power (dBW - Decibels relative to 1 Watt).
        % 'antenna_pattern_file': Path to CSV antenna gain pattern file (Gain in dBi assumed). Set to '' or leave field out for default omni.
        % 'offset_body_m': [X, Y, Z] position relative to aircraft CG (meters) in Body Frame (X-fwd, Y-right, Z-down).
        struct('id', 'C_Band_TX', 'freq_hz', 5.1e9, 'power_dbw', 10, 'antenna_pattern_file', 'data/antenna_patterns/c_band_pattern.csv', 'offset_body_m', [0, 0, -0.5]), ...
        struct('id', 'S_Band_TX', 'freq_hz', 2.4e9, 'power_dbw', 5, 'antenna_pattern_file', 'data/antenna_patterns/s_band_pattern.csv', 'offset_body_m', [0.5, 0, -0.5]) ...
        % Add more transmitters here if needed...
        % struct('id', 'X_Band_TX', 'freq_hz', 10e9, 'power_dbw', 15, 'antenna_pattern_file', '', 'offset_body_m', [0, 0.5, -0.5]) % Example Omni X-Band
    };

    % --- Ground Stations (Receivers) ---
    % Cell array, where each cell defines one ground station using a structure.
    config.rf.ground_stations = { ...
        % 'id': Unique string identifier for the ground station.
        % 'lat_deg', 'lon_deg', 'alt_m_msl': Geographic location (degrees, meters MSL).
        % 'rx_gain_dbi': Receiver antenna gain (dBi). Assumed constant/isotropic here.
        % 'sensitivity_dbm': Minimum received signal power threshold (dBm) for successful link.
        % 'bandwidth_hz': Receiver bandwidth (Hz). CRITICAL for noise power and capacity calculations.
        % 'supported_tx_ids': ** NEW ** Cell array {'TxID1', 'TxID2'} specifying which transmitter IDs this receiver can process.
        struct('id', 'GS1_SBand_Only', 'lat_deg', 35.64, 'lon_deg', -117.35, 'alt_m_msl', 2982.598, 'rx_gain_dbi', 20, 'sensitivity_dbm', -95, 'bandwidth_hz', 5e6, 'supported_tx_ids', {{'S_Band_TX'}}), ... % Listens ONLY to S_Band_TX
        struct('id', 'GS2_CBand_Only', 'lat_deg', 36.5739, 'lon_deg', -117.134, 'alt_m_msl', 4977.349, 'rx_gain_dbi', 25, 'sensitivity_dbm', -100, 'bandwidth_hz', 10e6, 'supported_tx_ids', {{'C_Band_TX'}}) ... % Listens ONLY to C_Band_TX
        % Add more ground stations here if needed...
        % struct('id', 'GS3_MultiBand', 'lat_deg', 34.60, 'lon_deg', -118.35, 'alt_m_msl', 400, 'rx_gain_dbi', 22, 'sensitivity_dbm', -98, 'bandwidth_hz', 8e6, 'supported_tx_ids', {{'S_Band_TX', 'X_Band_TX'}}) % Example listens to S & X
    };

    % --- RF Physics Parameters ---
    % Default bandwidth (Hz) used for noise calculations if 'bandwidth_hz' is missing for a ground station definition.
    config.rf.default_bandwidth_hz = 1e6; % 1 MHz

    % Receiver noise figure (dB). Represents noise added by the receiver itself. Higher value = noisier receiver.
    config.rf.noise_figure_db = 5.0;

    % System noise temperature (Kelvin). Used for calculating thermal noise floor (kTB). 290K is a standard ambient assumption.
    config.rf.noise_temperature_k = 290.0;

    % --- RF Loss Model Selection ---
    % Select model for atmospheric gas absorption loss.
    % 'none': No loss calculated.
    % 'simple_fixed': Very basic model based on keywords (from old code).
    % 'gas_itu': ** PLACEHOLDER - Requires implementing ITU-R P.676 model using atmospheric data. **
    config.rf.atmospheric_loss_model = 'gas_itu';

    % Select model for rain attenuation.
    % 'none': No rain loss calculated.
    % 'itu_rain': ** PLACEHOLDER - Requires implementing ITU-R P.838 model using rain rate data. **
    config.rf.rain_loss_model = 'none';

    % Fixed loss (dB) added to account for potential polarization mismatch between Tx and Rx antennas.
    config.rf.polarization_mismatch_loss_db = 0.0; % dB (Set > 0 if mismatch expected)

    % Toggle for enabling Fresnel zone clearance check (** requires implementation **).
    % If true, placeholder function `check_fresnel_clearance_placeholder` is called.
    config.rf.fresnel_zone_clearance_check = false;

    % Toggle for calculating Doppler shift based on relative velocity between Tx and Rx.
    config.rf.ENABLE_DOPPLER_SHIFT = true;

    % Toggle for including multipath effects (** requires implementation **).
    % If true, placeholder function `calculate_multipath_effects_placeholder` is called.
    config.rf.ENABLE_MULTIPATH_MODEL = false;
    % Selects which multipath placeholder logic to use ('none', 'simple_ricean', 'terrain_aware').
    config.rf.multipath_model_type = 'simple_ricean';
    % Ricean K-factor (dB) used by 'simple_ricean' placeholder (Ratio of direct path power to scattered power).
    config.rf.multipath_k_factor = 10.0; % dB

    % Toggle for future interference modeling (** requires implementation **).
    config.rf.ENABLE_INTERFERENCE_MODEL = false;


    %%%%% Need to clean
    config.rf.ENABLE_POINTING_ERROR        = false;
config.rf.pointing_error_std_dev_az_deg= 1.0;  % e.g. 1° std dev
config.rf.pointing_error_std_dev_el_deg= 0.5;  % e.g. 0.5° std dev

config.rf.ENABLE_GAS_LOSS             = true;
config.rf.ENABLE_RAIN_LOSS            = false;
config.rf.ENABLE_FOG_CLOUD_LOSS       = false;
config.rf.ENABLE_FRESNEL_LOSS         = true;
config.rf.ENABLE_XPD_LOSS             = true;
config.rf.ENABLE_GROUND_BOUNCE        = true;
config.rf.ENABLE_IONOSPHERIC_EFFECTS  = false;
config.rf.ENABLE_TROPO_DUCTING        = false;
config.rf.ENABLE_TERRAIN_DIFFRACTION  = false;
config.rf.ENABLE_MULTIPATH_FADING     = true;
config.rf.MULTIPATH_FADING_MODEL      = 'ricean';  % or 'rayleigh'
config.rf.RICEAN_K_FACTOR             = 6;         % dB
config.rf.ENABLE_SUMMARY_LOGGING      = true;

% Additional for Doppler Spread logic:
config.rf.ENABLE_DOPPLER_SPREAD_CALC  = true;
config.rf.DOPPLER_ANGLE_SPREAD_DEG    = 120;  % e.g. 120° angle spread

% Adaptive Modulation & BER:
config.rf.ENABLE_ADAPTIVE_MODULATION  = true;
config.rf.ENABLE_BER_SER_ESTIMATION   = true;
config.rf.ENABLE_SHANNON_CAPACITY     = true;

% ...
% Possibly in environment:
config.environment.pressure_hPa       = 1013.25;
config.environment.temperature_K       = 290;
config.environment.humidity_pct       = 50;
config.environment.rain_rate_mm_per_hr= 1.0; % light rain

    % =====================================================================
    % 8. Antenna Settings
    %    General settings related to antenna pattern handling.
    % =====================================================================
    config.antenna = struct();

    % Default isotropic gain (dBi) used if an antenna pattern file fails
    % to load, is not specified, or explicitly defines an omni pattern (e.g., empty file).
    config.antenna.default_gain_dbi = 0.0; % 0 dBi = Isotropic

    % Interpolation method used by `griddedInterpolant` for looking up gain
    % values between points in the antenna pattern grid.
    % Options: 'linear', 'nearest', 'cubic', 'spline'. 'linear' is usually sufficient.
    config.antenna.interpolation_method = 'linear';

    % =====================================================================
    % 9. Aircraft Masking Settings
    %    Configures how LOS checks against the aircraft body are performed.
    % =====================================================================
    config.masking = struct();

    % Master toggle for enabling checks for LOS blockage by the aircraft body itself.
    config.masking.ENABLE_AIRCRAFT_MASKING = true;

    % Method used for aircraft masking check:
    % 'none'      - No masking applied. LOS always clear w.r.t. aircraft body.
    % 'wireframe' - Uses a 2D polygon projection (fast, approximate). Requires `WIREFRAME_FILE`.
    % 'stl'       - Uses a 3D model and ray-tracing (accurate, slower). Requires `STL_FILE` and `stlread` function.
    config.masking.METHOD = 'wireframe';

    % ** IMPORTANT: Set path to your 2D wireframe definition file **
    % Expected format: CSV with X, Y columns defining polygon vertices in body frame (X-fwd, Y-right).
    config.masking.WIREFRAME_FILE = 'data/aircraft_models/aircraft_wireframe.csv'; % <<< SET YOUR FILE PATH HERE
    % Optionally preload points here to avoid file read, e.g.:
    % config.masking.WIREFRAME_POINTS = [-10,0; -5,5; 0,7.5; ... ; -10,0]; % Nx2 matrix

    % ** IMPORTANT: Set path to your 3D aircraft model file (.stl format). **
    config.masking.STL_FILE = 'data/aircraft_models/generic_fighter.stl'; % <<< SET YOUR FILE PATH HERE

    % Accuracy for transforming antenna offset when performing masking checks.
    % 'high' includes aircraft attitude (roll, pitch, heading) in the transform (more accurate).
    % 'simple' might ignore attitude (faster, less accurate, assumes offset relative to CG projected).
    config.masking.ANTENNA_OFFSET_TRANSFORM_ACCURACY = 'high'; % 'high' recommended

    % =====================================================================
    % 10. Radar Site (Reference Point for Look/Depression Angles)
    % =====================================================================
    config.radar = struct();
    % Identifier for the radar/reference site.
    config.radar.site_id = 'RADAR1';
    % Location of the reference point used for angle calculations.
    config.radar.lat_deg = 34.5;     % Latitude (degrees)
    config.radar.lon_deg = -118.0;   % Longitude (degrees)
    config.radar.alt_m_msl = 500;    % Altitude MSL (meters)

    % =====================================================================
    % 11. Visualization & Replay Settings
    %     Controls for plots, replay mode, real-time displays, and appearance.
    % =====================================================================
    config.visualization = struct();
    % Define position and size for figure windows [left, bottom, width, height] in pixels.
    % You can adjust these values or use MATLAB's default positioning.
    default_pos = get(groot,'DefaultFigurePosition'); % Get MATLAB's default
    config.visualization.FIGURE_POSITION = default_pos; % Use default position
    % Alternatively, set a custom position:
    % config.visualization.FIGURE_POSITION = [100, 100, 1000, 700]; % Example [left, bottom, width, height]

    % --- Master Toggles ---
    % Enable general real-time plotting updates. Specific real-time plots below
    % also need their individual flags set to true. If this is false, no real-time
    % plots will update, even if their specific flags are true.
    config.visualization.ENABLE_REALTIME_PLOTTING = false;
    % Generate summary plots after simulation completes using `visualization_model.m`.
    config.visualization.ENABLE_POST_SIM_PLOTS = true;
    % Enable the interactive 3D Tactical Replay mode (`tactical_replay_mode.m`).
    config.visualization.ENABLE_TACTICAL_REPLAY = false;

    % --- Appearance ---
    % Select visual style: 'standard' (MATLAB default) or 'cyberpunk'.
    config.visualization.PLOT_STYLE = 'cyberpunk';
    % Path to the theme definition file (used only if PLOT_STYLE is 'cyberpunk').
    config.visualization.CYBERPUNK_THEME_FILE = 'visualization/cyberpunk_theme.m';

    % --- Snapshot Settings ---
    % Interval (seconds) to automatically save snapshots of specified figures
    % if `config.output.ENABLE_SNAPSHOT_EXPORT` is true. Set to 0 or Inf to disable timed snapshots.
    config.visualization.SNAPSHOT_INTERVAL_S = 60;

    % --- Specific Plot Toggles & Settings ---
    % Toggle live rendering of 3D aircraft model during simulation (Patch 2).
    % ** Requires implementation of `realtime_3d_aircraft_update` function. **
    config.visualization.ENABLE_REALTIME_3D_AIRCRAFT = true;
    % Toggle live 3D plotting of flight path/map during simulation (Patch 3).
    % ** Requires implementation of `realtime_3d_flight_path_update` function. **
    config.visualization.ENABLE_REALTIME_3D_FLIGHT_PATH = true;
    % Toggle generation of the static ray trace plot (`ray_trace_visualizer.m`) at simulation end.
    config.visualization.ENABLE_RAY_TRACE_VISUALIZER = true;
    % Interval (seconds) for generating static ray trace snapshots during simulation
    % (requires SNAPSHOT_EXPORT to be true). Set high (e.g., Inf) to disable timed ray traces.
    config.visualization.RAY_TRACE_VIS_INTERVAL_S = Inf; % Disable timed ray traces by default
    % Toggle the real-time Look/Depression angle bin fill plot display during simulation.
    config.visualization.ENABLE_REALTIME_LOOKDEP_PLOT = false;
    % Toggle inclusion of terrain surface in 3D plots (trajectory, replay, ray trace).
    % Requires `config.terrain.ENABLE_TERRAIN_LOS` to be true and DEM loaded.
    config.visualization.ENABLE_TERRAIN_VIEW = true;

    % --- Plot Detail Settings ---
    % Scale factor (visual length in meters) for waypoint heading arrows in 3D plots (Patch 8).
    config.visualization.WAYPOINT_ARROW_SCALE = 500;
    % Default line width used in plots (can be overridden by theme).
    config.visualization.DEFAULT_LINE_WIDTH = 1.5;
    % Default MATLAB colormap name used for plots showing SNR values (e.g., scatter plot).
    config.visualization.SNR_COLORMAP = 'parula';
    % Default marker size used in plots (can be overridden by theme).
    config.visualization.PLOT_MARKER_SIZE = 6;
    % Height (as fraction of y-axis unit=1 link) for bars in the Link Status Timeline plot.
    config.visualization.LINK_STATUS_PLOT_HEIGHT = 0.15; % Make bars slightly thicker

    % --- Real-time Look/Depression Bin Fill Plot Specific Settings ---
    % Waypoint index to START collecting/plotting bin fill data.
    config.visualization.REALTIME_LOOKDEP_START_WP = 1;
    % Waypoint index to END collecting/plotting (set high, e.g., 999, to include all waypoints).
    config.visualization.REALTIME_LOOKDEP_END_WP = 999;
    % Azimuth (Look Angle) range for the heatmap plot (degrees).
    config.visualization.REALTIME_LOOKDEP_AZ_RANGE = [-180, 180];
    % Elevation (Depression Angle) range for the heatmap plot (degrees).
    config.visualization.REALTIME_LOOKDEP_EL_RANGE = [-90, 90];
    % Number of bins along the Azimuth axis in the heatmap.
    config.visualization.REALTIME_LOOKDEP_AZ_BINS = 90; % e.g., 4 degree bins for 360 range
    % Number of bins along the Elevation/Depression axis.
    config.visualization.REALTIME_LOOKDEP_EL_BINS = 45; % e.g., 4 degree bins for 180 range
    % Colormap name for the bin fill heatmap ('hot', 'jet', 'parula', 'cool', etc.).
    config.visualization.REALTIME_LOOKDEP_COLORMAP = 'hot';
    % Maximum update rate (Hz) for this real-time plot to limit performance impact.
    % Simulation step time `dt` also inherently limits the maximum possible rate.
    config.visualization.REALTIME_LOOKDEP_UPDATE_RATE_HZ = 5; % e.g., update plot max 5 times per second

    % =====================================================================
    % 12. Advanced / Debug Settings
    % =====================================================================
    config.advanced = struct();

    % Controls the amount of diagnostic text printed to the MATLAB command window during simulation.
    % 0 = Quiet (minimal output)
    % 1 = Normal (progress updates, key events)
    % 2 = Detailed Debugging (per-step info, warnings about adjustments, etc.)
    config.advanced.VERBOSE_LEVEL = 1;

    % If true, run `logic_dependency_checker.m` at the start to perform detailed
    % validation of configuration settings and dependencies. Recommended.
    config.advanced.PERFORM_LOGIC_CHECK = true;

    % If true, the simulation will attempt to run even if recommended toolboxes
    % (like Mapping Toolbox) are missing, relying on fallback methods where
    % available (e.g., CSV terrain loading instead of GeoTIFF). Note that fallbacks
    % might be less accurate or might fail if required data isn't in the fallback format.
    config.advanced.IGNORE_TOOLBOX_DEPENDENCIES = false;

    % =====================================================================
    % Configuration Loading Complete
    % =====================================================================
    disp('Configuration settings structure generated.');

    % Perform basic validation checks as a final step
    validate_configuration_helper(config); % Call internal helper function

    disp('Configuration settings loaded and validated.');

end % END OF MAIN FUNCTION config_settings


% =========================================================================
% INTERNAL HELPER: Basic Configuration Validation (Supplements logic_dependency_checker)
% =========================================================================
function validate_configuration_helper(config)
    % Performs simple, self-contained checks on configuration values.
    % More complex cross-dependency checks are done in logic_dependency_checker.m

    % --- Simulation Timing ---
    if ~isfield(config.simulation, 'time_step_s') || config.simulation.time_step_s <= 0
        error('Validation Error: config.simulation.time_step_s must be a positive number.');
    end
    if ~isfield(config.simulation, 'duration_s') || config.simulation.duration_s <= 0
        error('Validation Error: config.simulation.duration_s must be a positive number.');
    end

    % --- Waypoints ---
    if isempty(config.flight.waypoints)
        warning('Validation Warning: No waypoints defined in config.flight.waypoints. Aircraft behavior depends on keyboard input or holding initial state.');
    elseif size(config.flight.waypoints, 2) ~= 7
        error('Validation Error: config.flight.waypoints must have 7 columns: [Lat, Lon, AltFt, SpdKCAS, HdgCmd, PitchCmd, RollCmd]');
    end

    % --- File Paths (Basic Existence Check - more detailed in logic_checker) ---
    if config.terrain.ENABLE_TERRAIN_LOS && ~isfile(config.terrain.DEM_FILE_PATH)
        warning('Validation Warning: Terrain LOS enabled, but DEM file path seems invalid/missing: %s', config.terrain.DEM_FILE_PATH);
    end
    % Add similar basic checks for STL/Wireframe/Antenna files if desired,
    % although logic_dependency_checker handles the critical error checks.

    % --- Check for required config fields ---
    % Ensures critical fields haven't been accidentally deleted or commented out.
    required_fields = {'simulation.duration_s', 'output.BASE_FOLDER', ...
                       'flight.initial_state', 'flight.waypoints', ...
                       'rf.ENABLE_RF_MODULE', 'visualization.PLOT_STYLE'}; % Add more critical fields
    for i = 1:length(required_fields)
        try
            field_parts = strsplit(required_fields{i}, '.');
            temp = config;
            for j = 1:length(field_parts)
                temp = temp.(field_parts{j}); % Traverse struct path
            end
        catch
            error('Validation Error: Missing required config field path: %s', required_fields{i});
        end
    end

    disp('Basic configuration fields validated.');
end % END validate_configuration_helper