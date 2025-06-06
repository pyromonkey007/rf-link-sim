% =========================================================================
% ANTENNA MODEL LOADER (FINAL - Re-Commented)
% Version: FINAL 1.0
%
% Description:
%   Loads 3D antenna gain patterns for each transmitter specified in the
%   configuration file (`config.rf.transmitters`). It reads pattern data
%   from CSV files, assumes the data represents gain in dBi, creates
%   interpolation objects, and provides a function handle (`get_gain`)
%   for each antenna to allow real-time gain lookup based on azimuth and
%   elevation angles relative to the antenna's boresight.
%
% Pattern File Format (.csv):
%   - Expected format: A matrix of Gain values (in dBi).
%   - Rows correspond to Elevation angles.
%   - Columns correspond to Azimuth angles.
%   - Assumed Angle Ranges (can be adjusted if pattern metadata is available):
%     - Azimuth: 0 to 360 degrees (exclusive endpoint, wraps around).
%     - Elevation: -90 to +90 degrees.
%   - Mapping: Linear mapping from matrix index to angle based on grid size.
%   - Note: If the file contains Range (NM) instead of Gain (dBi), the code
%     has a basic (and potentially inaccurate) heuristic check and conversion
%     attempt based on inverse FSPL. **Providing Gain (dBi) files is strongly recommended.**
%
% Coordinate Frame for Lookup:
%   - The `get_gain(az_deg, el_deg)` function expects azimuth and elevation
%     angles relative to the antenna's boresight frame. The calling function
%     (`rf_propagation_model`) is responsible for calculating these angles
%     based on aircraft attitude, antenna mounting, and the LOS vector.
%
% Usage:
%   - Initialized in `main_simulation.m`:
%     `antennas = antenna_model_loader(config.rf.transmitters, config.antenna);`
%   - Gain lookup within `rf_propagation_model.m`:
%     `tx_gain_dbi = antennas.(tx_field_name).get_gain(az_body_deg, el_body_deg);`
%     (Assumes body frame = antenna frame in current implementation)
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-04]
% =========================================================================
function antennas_out = antenna_model_loader(transmitters_config, antenna_config)
    % Inputs:
    %   transmitters_config: Cell array of transmitter structs from config.rf.transmitters.
    %   antenna_config: Structure from config.antenna (contains default gain, interp method).

    fprintf('[ANT] Initializing Antenna Model Loader...\n');

    % Initialize the output structure which will hold data for each antenna.
    % The field names will be derived from the transmitter IDs.
    antennas_out = struct();

    % Check if any transmitters are defined in the configuration.
    if isempty(transmitters_config)
        warning('[ANT] No transmitters defined in config.rf.transmitters. Antenna loader returning empty.');
        return;
    end

    % --- Iterate through each transmitter definition in the config ---
    for i = 1:length(transmitters_config)
        tx_cfg = transmitters_config{i}; % Get the config struct for this transmitter
        tx_id  = tx_cfg.id;              % Get the unique ID string

        % Create a valid MATLAB field name from the ID (e.g., "C-Band TX" -> "C_Band_TX")
        tx_field_name = matlab.lang.makeValidName(tx_id);

        fprintf('  Loading antenna for Transmitter: "%s" (Struct field: %s)\n', tx_id, tx_field_name);

        % Initialize a structure to hold data for this specific antenna.
        ant_data = struct();
        ant_data.id             = tx_id;         % Store the original ID
        ant_data.config         = tx_cfg;        % Store transmitter-specific config (freq, power, offset)
        ant_data.pattern_loaded = false;       % Flag: True if pattern loaded successfully
        ant_data.interpolator   = [];          % Handle to the griddedInterpolant object
        ant_data.az_vector      = [];          % Vector of azimuth angles for the grid
        ant_data.el_vector      = [];          % Vector of elevation angles for the grid
        ant_data.gain_matrix_dbi= [];          % The loaded gain data

        % --- Get Antenna Pattern File Path ---
        if ~isfield(tx_cfg, 'antenna_pattern_file') || isempty(tx_cfg.antenna_pattern_file)
            % If no pattern file is specified, use the default omni gain.
            warning('[ANT %s] No antenna_pattern_file specified. Using default omni gain %.1f dBi.', tx_id, antenna_config.default_gain_dbi);
            gain_val = antenna_config.default_gain_dbi;
            % Create a simple function handle that always returns the default gain.
            ant_data.get_gain = @(az, el) gain_val;
            antennas_out.(tx_field_name) = ant_data; % Store data for this antenna
            continue; % Skip to the next transmitter
        end
        pattern_file = tx_cfg.antenna_pattern_file;

        % --- Load Pattern File ---
        if isfile(pattern_file)
            try
                fprintf('    Reading pattern file: %s\n', pattern_file);
                % Read the CSV matrix. Assumes numeric data.
                gain_matrix_or_range = readmatrix(pattern_file);

                % --- Check if data looks like Gain (dBi) or Range (NM) ---
                % This is a simple heuristic and might be incorrect.
                % It's better to ensure CSV files contain Gain (dBi).
                is_likely_range = all(gain_matrix_or_range(:) > 0) && max(gain_matrix_or_range(:)) > 50; % Arbitrary threshold

                if is_likely_range
                    % Attempt conversion from Range (NM) to dBi using inverse FSPL.
                    % ** WARNING: This conversion is highly dependent on assumed link budget **
                    % ** parameters (like sensitivity) used when the range data was generated. **
                    % ** Results may be inaccurate. Use Gain (dBi) files for reliability. **
                    fprintf('    [!] Warning: Pattern file appears to contain Range(NM). Attempting rough conversion to dBi via inverse FSPL.\n');
                    dist_nm = gain_matrix_or_range;
                    dist_m  = dist_nm * 1852; % Convert NM to meters
                    freq_hz = tx_cfg.freq_hz;
                    max_range_m = max(dist_m, [], 'all');
                    if isempty(freq_hz) || freq_hz <= 0
                        error('Transmitter frequency (tx_cfg.freq_hz) is invalid, needed for Range->Gain conversion.');
                    end
                    % FSPL(dB) = 20*log10(d_m) + 20*log10(f_hz) - 147.55
                    % Inverse FSPL: Gain ~ -FSPL + Constant Offset (unknown)
                    peak_gain = 2;
                    gain_dBi = peak_gain + 20*log10(dist_m/max_range_m);
                    warning('Range->Gain conversion accuracy depends heavily on original link assumptions.');
                else
                    % Assume the file contains Gain in dBi directly.
                    fprintf('    Assuming pattern file contains Gain (dBi).\n');
                    gain_dBi = gain_matrix_or_range;
                end

                % Store the final gain matrix (either read directly or converted).
                ant_data.gain_matrix_dbi = gain_dBi;

                % --- Create Azimuth and Elevation Angle Vectors ---
                [num_el_points, num_az_points] = size(gain_dBi);
                if num_el_points < 2 || num_az_points < 2
                    error('Antenna pattern matrix must be at least 2x2.');
                end

                % Assume linear spacing for angles based on grid size.
                % Azimuth: 0 to <360 degrees (exclusive endpoint for wrapping).
                az_vector = linspace(0, 360, num_az_points + 1);
                az_vector = az_vector(1:end-1); % e.g., [0, 10, ..., 350]

                % Elevation: -90 to +90 degrees (inclusive).
                el_vector = linspace(-90, 90, num_el_points);

                ant_data.az_vector = az_vector;
                ant_data.el_vector = el_vector;

                % --- Create Interpolation Object ---
                % `griddedInterpolant` requires monotonic grid vectors and matching matrix dimensions.
                % We need to handle the azimuth wraparound (0 = 360 degrees).
                [az_grid, el_grid] = meshgrid(az_vector, el_vector);

                % Pad the data for seamless wrapping interpolation around azimuth.
                % Repeat the first column of gain data at the end (representing 360 deg).
                gain_padded = [gain_dBi, gain_dBi(:, 1)];
                % Extend the azimuth grid vector to include 360 deg.
                az_grid_padded = [az_grid, az_grid(:, 1) + 360];
                % Repeat the elevation grid to match dimensions.
                el_grid_padded = [el_grid, el_grid(:, 1)];

                % Create the interpolant object using the padded grid and data.
                % 'none' extrapolation returns NaN for points outside the El range.
                ant_data.interpolator = griddedInterpolant(el_grid_padded, az_grid_padded, gain_padded, ...
                                                          antenna_config.interpolation_method, 'none');

                ant_data.pattern_loaded = true; % Mark as successfully loaded
                fprintf('    Pattern loaded: %d El points [%.1f to %.1f deg], %d Az points [%.1f to %.1f deg].\n', ...
                        num_el_points, min(el_vector), max(el_vector), ...
                        num_az_points, min(az_vector), max(az_vector));

                % Assign the function handle for gain lookup using this antenna's data.
                ant_data.get_gain = @(az, el) lookup_gain_internal(ant_data, az, el, antenna_config.default_gain_dbi);

            catch ME_read % Handle errors during file reading or processing
                warning('[ANT %s] Pattern file "%s" load/process error: %s. Using default omni gain.', ...
                        tx_id, pattern_file, ME_read.message);
                gain_val = antenna_config.default_gain_dbi;
                ant_data.get_gain = @(az, el) gain_val; % Fallback to default gain function
            end
        else
            % If pattern file doesn't exist.
            warning('[ANT %s] Pattern file not found: "%s". Using default omni gain %.1f dBi.', ...
                    tx_id, pattern_file, antenna_config.default_gain_dbi);
            gain_val = antenna_config.default_gain_dbi;
            ant_data.get_gain = @(az, el) gain_val; % Fallback to default gain function
        end

        % Store the data structure for this antenna in the main output structure.
        antennas_out.(tx_field_name) = ant_data;

    end % End loop through transmitters

    fprintf('[ANT] Antenna model loader initialization complete.\n');

end % END OF FUNCTION antenna_model_loader


% =========================================================================
% INTERNAL HELPER: Gain Lookup Function
% =========================================================================
function gain_dBi = lookup_gain_internal(ant_data, az_deg, el_deg, default_gain)
    % Performs gain lookup using the pre-computed interpolator for a specific antenna.
    % Handles azimuth wrapping and elevation clamping.
    % Inputs:
    %   ant_data     - Structure containing the specific antenna's interpolator, vectors, etc.
    %   az_deg       - Azimuth angle relative to antenna boresight (degrees).
    %   el_deg       - Elevation angle relative to antenna boresight (degrees).
    %   default_gain - Gain value to return on failure or if pattern not loaded (dBi).
    % Output:
    %   gain_dBi     - Interpolated antenna gain (dBi).

    % --- Check if pattern was loaded and interpolator exists ---
    if ~ant_data.pattern_loaded || isempty(ant_data.interpolator)
        gain_dBi = default_gain;
        return;
    end

    % --- Input Validation ---
    if ~isnumeric(az_deg) || ~isscalar(az_deg) || ~isnumeric(el_deg) || ~isscalar(el_deg)
        warning('[ANT %s] Gain lookup requires numeric scalar Az/El inputs.', ant_data.id);
        gain_dBi = default_gain;
        return;
    end

    % --- Angle Normalization ---
    % Wrap Azimuth to the [0, 360) range used by the padded interpolator grid.
    az_lookup = mod(az_deg, 360);

    % Clamp Elevation to the pattern's vertical limits (e.g., -90 to +90).
    el_min = min(ant_data.el_vector);
    el_max = max(ant_data.el_vector);
    el_lookup = max(min(el_deg, el_max), el_min);
    % Optional: Warn if elevation was clamped significantly?
    % if abs(el_lookup - el_deg) > 1.0 % If clamped by more than 1 degree
    %     fprintf('Debug [Ant %s]: Clamped elevation %.1f to %.1f deg for lookup.\n', ant_data.id, el_deg, el_lookup);
    % end

    % --- Interpolation ---
    try
        % Perform interpolation using Elevation (row index) then Azimuth (column index).
        gain_dBi = ant_data.interpolator(el_lookup, az_lookup);

        % Handle NaN results (e.g., if using 'none' extrapolation and query is outside El range).
        if isnan(gain_dBi)
            gain_dBi = default_gain;
            % Warning for NaN is often too noisy if pattern edges are sharp. Enable if needed for debug.
            % warning('[ANT %s] Interpolation resulted in NaN for Az=%.1f, El=%.1f. Using default.', ant_data.id, az_deg, el_deg);
        end

    catch ME_interp % Catch errors during interpolation call
        warning('[ANT %s] Gain interpolation failed for Az=%.1f, El=%.1f: %s. Using default gain.', ...
                ant_data.id, az_deg, el_deg, ME_interp.message);
        gain_dBi = default_gain;
    end

end % END OF FUNCTION lookup_gain_internal