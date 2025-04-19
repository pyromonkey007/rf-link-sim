% =========================================================================
% RF PROPAGATION MODEL (High Fidelity w/ Toolbox Integration & Masking)
% Version: 2.1
%
% Description:
%   Calculates the RF link budget and performance metrics for
%   moving aircraft transmitter and stationary ground station receiver pairs
%   within a WGS84 coordinate system. Includes aircraft masking checks.
%   Conditionally utilizes MATLAB toolboxes (Communications, Antenna, Mapping)
%   for improved accuracy in propagation modeling if available, otherwise
%   falls back to manual calculations/placeholders.
%
% Core Functionality:
%   - Checks for MATLAB Toolbox availability.
%   - Iterates through Tx-Rx pairs, checks band support.
%   - Geometry Calculation (Range, LOS vectors in ECEF).
%   - Antenna Gain Lookup (using nominal body angles).
%   - LOS Checks:
%       - Terrain LOS (using toolboxes or manual).
%       - Aircraft Masking (using STL/Wireframe if enabled & not terrain blocked).
%   - Conditional Propagation Loss Calculation:
%       - If Toolboxes: Uses fspl, gaspl, rainpl, fogpl, los.
%       - Else: Uses manual formulas/placeholders.
%       - Calculates: FSPL, Gas Loss, Rain Loss, Fog/Cloud Loss,
%                     Diffraction (basic), Ground Bounce (basic).
%       - Excludes: Pointing Error, XPD, Ionospheric, Tropospheric losses.
%   - Received Power, Noise Power, SNR calculation.
%   - Link Margin, Adaptive Modulation selection.
%   - BER Estimation.
%   - Doppler Shift Calculation (basic).
%   - Shannon Capacity Calculation.
%   - Status & Failure Reason Logging.
%   - Addresses potential "dissimilar structures" error by ensuring consistent
%     struct field population before assignment.
%
% Inputs:
%   config     - Full configuration structure.
%   state      - Aircraft state (including LLH, attitude, velocity).
%   earth      - Earth model object (for coordinate transforms).
%   atmosphere - Atmospheric model object (provides temp, pressure etc.).
%   terrain    - Terrain model object (provides check_LOS and potentially data).
%   antennas   - Antenna models structure (provides get_gain).
%   t_sim      - Simulation time (seconds).
%
% Outputs:
%   rf_status       - Detailed results structure array for each potential link.
%   failure_summary - Summary struct of link failure reasons and counts.
%
% Maintainer: [Your Name/Team] - Based on Gemini V2.0 + User Request
% Last Updated: 2025-04-14
% =========================================================================
function rf_model_obj = rf_propagation_model()
    % Constructor for the RF model object.
    % Returns a structure with a handle to the main computation function.
    rf_model_obj.compute_rf = @compute_rf_links_internal;
    fprintf('[RF MODEL] RF Propagation Model Initialized (v2.1 - Toolbox Integration & Masking).\n');
end % END OF CONSTRUCTOR

% =========================================================================
% INTERNAL FUNCTION: COMPUTE RF LINKS
% =========================================================================
function [rf_status, failure_summary] = compute_rf_links_internal(config, state, earth, atmosphere, terrain, antennas, t_sim)

    % --- Constants ---
    C_LIGHT     = 299792458;    % Speed of light (m/s)
    K_BOLTZMANN = 1.380649e-23;   % Boltzmann constant (J/K)
    DEG2RAD     = pi/180;
    RAD2DEG     = 180/pi;

    % --- Basic Simulation Sanity Checks ---
    if ~isfield(config, 'rf') || ~isfield(config.rf, 'ENABLE_RF_MODULE') || ~config.rf.ENABLE_RF_MODULE
        warning('[RF MODEL] RF Module is disabled in configuration.');
        rf_status=struct('status','RF Disabled');
        failure_summary=struct('reason','RF Disabled');
        return;
    end
    % Check required state fields more thoroughly
    required_state_fields = {'lat_deg', 'lon_deg', 'alt_m_msl', 'velocity_ned_mps', 'heading_deg', 'pitch_deg', 'roll_deg'};
    missing_fields = required_state_fields(~isfield(state, required_state_fields));
    if ~isempty(missing_fields)
        error('RF Model requires state fields: %s', strjoin(missing_fields, ', '));
    end
     if isempty(state.velocity_ned_mps) || ~isnumeric(state.velocity_ned_mps) || numel(state.velocity_ned_mps) ~= 3
        error('RF Model requires state.velocity_ned_mps to be a 3-element numeric vector.');
    end
    if isempty(config.rf.transmitters) || isempty(config.rf.ground_stations)
        warning('[RF MODEL] No Transmitters or Ground Stations defined in configuration.');
        rf_status=struct('status','No Tx/Rx'); failure_summary=struct('reason','No Tx/Rx'); return;
    end

    % --- Check Toolbox Availability ---
    % Store results in a structure for easy access
    toolboxes.comm    = license('test', 'communications_toolbox') && ~isempty(ver('comm'));
    toolboxes.antenna = license('test', 'antenna_toolbox') && ~isempty(ver('antenna'));
    toolboxes.phased  = license('test', 'phased_array_sys_toolbox') && ~isempty(ver('phased')); % Although not used directly for losses here yet
    toolboxes.mapping = license('test', 'map_toolbox') && ~isempty(ver('map'));
    % Determine if *any* relevant toolbox for propagation modeling is available
    toolboxes.has_any_prop_toolbox = toolboxes.comm || toolboxes.antenna || toolboxes.mapping;

    if mod(t_sim, 10) < config.simulation.time_step_s || t_sim < config.simulation.time_step_s % Print status only occasionally
     fprintf('[RF MODEL @ t=%.1fs] Toolbox Status: Comm=%d, Antenna=%d, Mapping=%d\n', ...
             t_sim, toolboxes.comm, toolboxes.antenna, toolboxes.mapping);
    end

    % --- Initialize Output Structures ---
    num_tx = length(config.rf.transmitters);
    num_rx = length(config.rf.ground_stations);
    num_links = num_tx * num_rx;
    % Pre-allocate the results structure array using the helper function
    % This ensures all fields exist from the start, preventing dissimilar struct errors.
    rf_status = repmat(create_link_status_struct_helper(), num_links, 1); % UPDATED HELPER CALL
    link_idx = 0; % Linear index into the rf_status array
    failure_summary = struct(); % Initialize summary structure

    % --- Calculate Aircraft ECEF State (once per timestep) ---
    try
        [ac_pos_ecef(1), ac_pos_ecef(2), ac_pos_ecef(3)] = earth.LLH_to_ECEF(state.lat_deg, state.lon_deg, state.alt_m_msl);
        R_ned2ecef = earth.NED_to_ECEF_matrix(state.lat_deg, state.lon_deg); % Rotation from local NED to ECEF
        ac_vel_ecef = R_ned2ecef * state.velocity_ned_mps(:); % Aircraft velocity in ECEF
    catch ME_ecef
        error('Failed to calculate aircraft ECEF state at t=%.1f s: %s', t_sim, ME_ecef.message);
    end

    % =====================================================================
    % === Loop Through Each Transmitter (on Aircraft) ===
    % =====================================================================
    for i_tx = 1:num_tx
        tx_cfg = config.rf.transmitters{i_tx};
        tx_id = tx_cfg.id;
        tx_field_name = matlab.lang.makeValidName(tx_id); % For accessing antenna model

        % --- Check Antenna Model Exists ---
        if ~isfield(antennas, tx_field_name)
            warning('[RF MODEL @ t=%.1f] Antenna model for Tx "%s" not found. Skipping this transmitter.', t_sim, tx_id);
            % Populate status for skipped links due to this Tx
            for i_rx_skip = 1:num_rx
                link_idx = link_idx + 1;
                rx_cfg_skip = config.rf.ground_stations{i_rx_skip};
                rf_status(link_idx).time_s = t_sim;
                rf_status(link_idx).tx_id = tx_id;
                rf_status(link_idx).rx_id = rx_cfg_skip.id;
                rf_status(link_idx).status = 'SKIP';
                rf_status(link_idx).failure_reason = 'Antenna Model Missing';
                rf_status(link_idx).primary_failure_step = 'Setup';
            end
            continue; % Move to the next transmitter
        end
        ant_model = antennas.(tx_field_name);
        freq_hz = tx_cfg.freq_hz;
        tx_power_dbw = tx_cfg.power_dbw; % Power in dBW
        tx_offset_body = tx_cfg.offset_body_m(:); % Ensure column vector

        % --- Calculate Transmitter's Precise ECEF Position (considering body offset) ---
        try
            % Get body-to-NED rotation matrix from aircraft attitude
            R_body2ned = angle2dcm_local(state.heading_deg*DEG2RAD, state.pitch_deg*DEG2RAD, state.roll_deg*DEG2RAD, 'ZYX');
            % Rotate body offset vector first to NED, then to ECEF
            tx_offset_ecef = R_ned2ecef * R_body2ned * tx_offset_body;
            % Add offset to aircraft center ECEF position
            tx_pos_ecef = ac_pos_ecef(:) + tx_offset_ecef; % Ensure column vector arithmetic
        catch ME_tx_pos
            warning('[RF MODEL @ t=%.1f] Tx "%s" Failed to calculate ECEF position: %s. Skipping Tx.', t_sim, tx_id, ME_tx_pos.message);
             % Populate status for skipped links
            remaining_rx_indices = (link_idx + 1):(i_tx * num_rx); % Indices for remaining Rxs for this Tx
            for current_link_idx = remaining_rx_indices
                 rx_idx = mod(current_link_idx - 1, num_rx) + 1;
                 rx_cfg_skip = config.rf.ground_stations{rx_idx};
                 rf_status(current_link_idx).time_s=t_sim;
                 rf_status(current_link_idx).tx_id=tx_id;
                 rf_status(current_link_idx).rx_id=rx_cfg_skip.id;
                 rf_status(current_link_idx).status='SKIP';
                 rf_status(current_link_idx).failure_reason='Tx Position Error';
                 rf_status(current_link_idx).primary_failure_step='Setup';
            end
            link_idx = i_tx * num_rx; % Update linear index to end of this Tx block
            continue; % Move to the next transmitter
        end

        % =====================================================================
        % === Loop Through Each Ground Station (Receiver) ===
        % =====================================================================
        for i_rx = 1:num_rx
            link_idx = link_idx + 1; % Increment linear link index
            rx_cfg = config.rf.ground_stations{i_rx};
            rx_id = rx_cfg.id;

            % --- Initialize Status Struct for This Specific Link ---
            % Ensures all fields exist and are initialized before any calculation
            link_status = create_link_status_struct_helper(); % UPDATED HELPER CALL
            link_status.time_s = t_sim;
            link_status.tx_id = tx_id;
            link_status.rx_id = rx_id;
            link_status.frequency_hz = freq_hz;
            link_status.tx_power_dbm = tx_power_dbw + 30; % Store Tx power in dBm
            link_status.status = 'PENDING'; % Initial status

            % --- Band Filtering Check ---
            % Check if this receiver is configured to support this transmitter
            supports_tx = false;
            if isfield(rx_cfg, 'supported_tx_ids') && iscellstr(rx_cfg.supported_tx_ids)
                if ismember(tx_id, rx_cfg.supported_tx_ids)
                    supports_tx = true;
                end
            end
            if ~supports_tx
                link_status.status='SKIP';
                link_status.failure_reason='Rx !support Tx';
                link_status.primary_failure_step='Setup';
                rf_status(link_idx)=link_status; % Store the updated status
                continue; % Move to the next receiver
            end

            % --- If supported, proceed with detailed link calculation ---
            link_status.status = 'OK'; % Assume OK initially

            try % Wrap the entire calculation for this link in a try-catch block

                % --- 1. Calculate Geometry & Basic Parameters ---
                % Get receiver ECEF position (stationary)
                [rx_pos_ecef(1), rx_pos_ecef(2), rx_pos_ecef(3)] = earth.LLH_to_ECEF(rx_cfg.lat_deg, rx_cfg.lon_deg, rx_cfg.alt_m_msl);
                % Calculate line-of-sight (LOS) vector in ECEF (from Tx to Rx)
                los_vector_ecef = rx_pos_ecef(:) - tx_pos_ecef(:);
                % Calculate slant range
                range_m = norm(los_vector_ecef);
                link_status.range_m = range_m;

                % Check for trivial range case
                if range_m < 1.0 % Avoid division by zero or nonsensical results
                    link_status.status='FAIL';
                    link_status.failure_reason='Range < 1m';
                    link_status.primary_failure_step='Geometry';
                    rf_status(link_idx)=link_status; % Store updated status
                    continue; % Move to next receiver
                end
                % Calculate LOS unit vector in ECEF
                los_unit_ecef = los_vector_ecef / range_m;
                % Calculate wavelength
                lambda_m = C_LIGHT / freq_hz;

                % --- 2. Coordinate Transformations for Antenna Pointing & Masking ---
                % Transform LOS vector from ECEF to aircraft's NED frame
                R_ecef2ned_ac = earth.ECEF_to_NED_matrix(state.lat_deg, state.lon_deg);
                los_vector_ned = R_ecef2ned_ac * los_vector_ecef;
                % Transform LOS vector from aircraft's NED frame to its body frame
                R_body2ned = angle2dcm_local(state.heading_deg*DEG2RAD, state.pitch_deg*DEG2RAD, state.roll_deg*DEG2RAD, 'ZYX'); % Get Body->NED needed below
                R_ned2body = R_body2ned'; % Transpose for NED to Body
                los_vector_body = R_ned2body * los_vector_ned;

                % Calculate nominal Azimuth and Elevation angles in the aircraft body frame
                % Azimuth (deg): 0 is fwd, positive clockwise. Range [0, 360)
                az_body_nominal = mod(atan2d(los_vector_body(2), los_vector_body(1)), 360);
                % Elevation (deg): Positive is upwards from body XY plane. Range [-90, 90]
                el_body_nominal = atan2d(-los_vector_body(3), sqrt(los_vector_body(1)^2 + los_vector_body(2)^2)); % Note: -Z_body is typically down

                link_status.azimuth_body_deg = az_body_nominal;   % Store nominal look angles
                link_status.elevation_body_deg = el_body_nominal;

                % --- 3. Antenna Gain Lookup (Using Nominal Angles) ---
                % Note: Pointing error loss is excluded as requested. Using nominal angles.
                tx_gain_dbi = ant_model.get_gain(az_body_nominal, el_body_nominal);
                rx_gain_dbi = rx_cfg.rx_gain_dbi; % Assuming fixed Rx gain from config
                total_antenna_gain_dbi = tx_gain_dbi + rx_gain_dbi;
                link_status.tx_gain_dbi = tx_gain_dbi;
                link_status.rx_gain_dbi = rx_gain_dbi;

                % --- Retrieve Atmospheric Conditions at Aircraft Altitude ---
                % Needed for both toolbox and manual gas loss calculations
                atmos_cond = atmosphere.get_conditions(state.alt_m_msl);
                temperature_k = atmos_cond.temperature_k;
                pressure_pa = atmos_cond.pressure_pa;
                % Placeholder for water vapor density if needed by gaspl or manual calc
                rho_w = 1.0; % Example: 1 g/m^3. Ideally get from config or humidity.

                % --- 4. Propagation Loss Calculations ---
                % Initialize all loss components included in this version
                fspl_db = NaN;
                gas_loss_db = 0;
                rain_loss_db = 0;
                fog_cloud_loss_db = 0;
                diffraction_loss_db = 0;
                ground_bounce_loss_db = 0; % Manual placeholder calculation will be used
                is_terrain_blocked = false; % Initialize LOS flag
                is_aircraft_masked = false; % Initialize masking flag

                % Get Tx/Rx positions in LLH for propagation models if needed
                tx_lat = NaN; tx_lon = NaN; tx_alt = NaN; % Initialize to NaN
                try
                    [tx_lat, tx_lon, tx_alt] = earth.ECEF_to_LLH(tx_pos_ecef(1), tx_pos_ecef(2), tx_pos_ecef(3));
                    % *** ADD VALIDATION HERE ***
                    if isempty(tx_lat) || ~isscalar(tx_lat) || ~isfinite(tx_lat) || ...
                       isempty(tx_lon) || ~isscalar(tx_lon) || ~isfinite(tx_lon) || ...
                       isempty(tx_alt) || ~isscalar(tx_alt) || ~isfinite(tx_alt)
                        warning('[RF %s->%s @ t=%.1f] Invalid coordinates returned from ECEF_to_LLH for Tx. Skipping Toolbox path this step.', tx_id, rx_id, t_sim);
                        force_manual_calc = true; % Set a flag to force manual calculation below
                    else
                        force_manual_calc = false; % Coordinates seem valid
                    end
                     % *** END ADDED VALIDATION ***
                catch ME_llh
                    warning('[RF %s->%s @ t=%.1f] Error during ECEF_to_LLH conversion: %s. Skipping Toolbox path this step.', tx_id, rx_id, t_sim, ME_llh.message);
                    force_manual_calc = true; % Force manual on error
                end
                rx_lat = rx_cfg.lat_deg; rx_lon = rx_cfg.lon_deg; rx_alt = rx_cfg.alt_m_msl;

                % --- 4.1. Select calculation method based on toolbox availability ---
                if toolboxes.has_any_prop_toolbox && ~force_manual_calc
                    % --- TOOLBOX ASSISTED CALCULATIONS ---
                    sites_created_ok = false; % Flag to track if site objects are valid
                    txs = []; rxs = []; % Initialize

                    % --- Validate Inputs BEFORE creating site objects ---
                    inputs_valid = true;
                    if isempty(tx_id) || ~ischar(tx_id), inputs_valid = false; warning('[RF VALIDATION %s->%s @ t=%.1f] tx_id invalid.', tx_id, rx_id, t_sim); end
                    if isempty(tx_lat) || ~isscalar(tx_lat) || ~isfinite(tx_lat), inputs_valid = false; warning('[RF VALIDATION %s->%s @ t=%.1f] tx_lat invalid.', tx_id, rx_id, t_sim); end
                    if isempty(tx_lon) || ~isscalar(tx_lon) || ~isfinite(tx_lon), inputs_valid = false; warning('[RF VALIDATION %s->%s @ t=%.1f] tx_lon invalid.', tx_id, rx_id, t_sim); end
                    if isempty(tx_alt) || ~isscalar(tx_alt) || ~isfinite(tx_alt), inputs_valid = false; warning('[RF VALIDATION %s->%s @ t=%.1f] tx_alt invalid.', tx_id, rx_id, t_sim); end
                    if isempty(rx_id) || ~ischar(rx_id), inputs_valid = false; warning('[RF VALIDATION %s->%s @ t=%.1f] rx_id invalid.', tx_id, rx_id, t_sim); end
                    if isempty(rx_lat) || ~isscalar(rx_lat) || ~isfinite(rx_lat), inputs_valid = false; warning('[RF VALIDATION %s->%s @ t=%.1f] rx_lat invalid.', tx_id, rx_id, t_sim); end
                    if isempty(rx_lon) || ~isscalar(rx_lon) || ~isfinite(rx_lon), inputs_valid = false; warning('[RF VALIDATION %s->%s @ t=%.1f] rx_lon invalid.', tx_id, rx_id, t_sim); end
                    if isempty(rx_alt) || ~isscalar(rx_alt) || ~isfinite(rx_alt), inputs_valid = false; warning('[RF VALIDATION %s->%s @ t=%.1f] rx_alt invalid.', tx_id, rx_id, t_sim); end

                    % --- Attempt to create site objects only if inputs are valid and toolboxes exist ---
                    if inputs_valid && (toolboxes.antenna || toolboxes.comm)
                         try
                             % % AntennaHeight is height above ground/terrain. Using altitude MSL might
                             % % require terrain data for accurate relative height. For simplicity,
                             % % using MSL but be aware this might not be correct for some models.
                             % txsite = txsite('Name', tx_id, 'Latitude', tx_lat, 'Longitude', tx_lon, 'AntennaHeight', tx_alt);
                             % rxsite = rxsite('Name', rx_id, 'Latitude', rx_lat, 'Longitude', rx_lon, 'AntennaHeight', rx_alt);
                             % sites_created_ok = true; % Mark as successful

                             txs = txsite("Name", tx_id, "Latitude", tx_lat, "Longitude", tx_lon, "AntennaHeight", tx_alt); % DEBUG: Use fixed height
                             rxs = rxsite("Name", rx_id, "Latitude", rx_lat, "Longitude", rx_lon, "AntennaHeight", rx_alt); % DEBUG: Use fixed height
                             sites_created_ok = true; % Mark as successful if this runs without error
                        

                         catch ME_site
                             warning('[RF %s->%s @ t=%.1f] Failed to create site objects: %s. May impact toolbox calcs.', tx_id, rx_id, t_sim, ME_site.message);
                             sites_created_ok = false; % Explicitly mark as failed
                         end
                    elseif ~inputs_valid
                         warning('[RF %s->%s @ t=%.1f] Skipping site object creation due to invalid inputs.', tx_id, rx_id, t_sim);
                         sites_created_ok = false;
                    else
                         % Toolboxes needed are not available
                         sites_created_ok = false;
                    end % End site creation block

                    % --- Proceed with Toolbox Calcs ONLY if sites were created successfully ---
                    if sites_created_ok % Use the flag created above
                        % --- a) Free Space Path Loss (FSPL) ---
                        if toolboxes.antenna
                            try
                                fspl_db = fspl(range_m, lambda_m);
                            catch ME_fspl
                                warning('[RF %s->%s @ t=%.1f] Antenna Toolbox fspl failed: %s. Using manual.', tx_id, rx_id, t_sim, ME_fspl.message);
                                fspl_db = calculate_manual_fspl(range_m, freq_hz);
                            end
                        else
                            fspl_db = calculate_manual_fspl(range_m, freq_hz);
                        end

                        % --- b) Atmospheric Gas Loss ---
                        if toolboxes.comm
                            try
                                % gaspl(distance, frequency, temperature, pressure, waterVapourDensity)
                                gas_loss_db = gaspl(range_m, freq_hz, temperature_k, pressure_pa, rho_w);
                            catch ME_gas
                                warning('[RF %s->%s @ t=%.1f] Comm Toolbox gaspl failed: %s. Using manual.', tx_id, rx_id, t_sim, ME_gas.message);
                                gas_loss_db = calculate_manual_gas_loss(range_m, freq_hz, temperature_k, pressure_pa, rho_w);
                            end
                        else
                             gas_loss_db = calculate_manual_gas_loss(range_m, freq_hz, temperature_k, pressure_pa, rho_w);
                        end

                        % --- c) Rain Loss ---
                        if toolboxes.comm && isfield(config.environment, 'rain_rate_mm_per_hr') && config.environment.rain_rate_mm_per_hr > 0
                            try
                                rain_rate = config.environment.rain_rate_mm_per_hr;
                                % rainpl(distance, frequency, rainRate, elevation, [tau]) - tau is polarization tilt
                                el_deg_ground = asind((tx_alt - rx_alt) / range_m); % Simple elevation approx
                                rain_loss_db = rainpl(range_m, freq_hz, rain_rate, el_deg_ground);
                             catch ME_rain
                                warning('[RF %s->%s @ t=%.1f] Comm Toolbox rainpl failed: %s. Using manual.', tx_id, rx_id, t_sim, ME_rain.message);
                                rain_loss_db = calculate_manual_rain_loss(range_m, freq_hz, rain_rate, el_deg_ground);
                            end
                        else
                            % Use manual only if rain is configured but toolbox unavailable
                            if isfield(config.environment, 'rain_rate_mm_per_hr') && config.environment.rain_rate_mm_per_hr > 0
                                rain_rate = config.environment.rain_rate_mm_per_hr;
                                el_deg_ground = asind((tx_alt - rx_alt) / range_m); % Simple elevation approx
                                 rain_loss_db = calculate_manual_rain_loss(range_m, freq_hz, rain_rate, el_deg_ground);
                            else
                                rain_loss_db = 0; % No rain configured
                            end
                        end

                        % --- d) Fog / Cloud Loss ---
                         if toolboxes.comm && isfield(config.environment, 'liquid_water_density') && config.environment.liquid_water_density > 0
                            try
                                liquid_water_density = config.environment.liquid_water_density; % kg/m^3 or g/m^3? Check fogpl doc
                                % fogpl(distance, frequency, liquidWaterDensity, temperature)
                                fog_cloud_loss_db = fogpl(range_m, freq_hz, liquid_water_density, temperature_k);
                             catch ME_fog
                                warning('[RF %s->%s @ t=%.1f] Comm Toolbox fogpl failed: %s. Using manual.', tx_id, rx_id, t_sim, ME_fog.message);
                                fog_cloud_loss_db = calculate_manual_fog_loss(range_m, freq_hz); % Basic placeholder
                            end
                        else
                            % Use manual only if fog is configured but toolbox unavailable
                             if isfield(config.environment, 'liquid_water_density') && config.environment.liquid_water_density > 0
                                fog_cloud_loss_db = calculate_manual_fog_loss(range_m, freq_hz); % Basic placeholder
                             else
                                 fog_cloud_loss_db = 0; % No fog configured
                             end
                        end

                       % --- e) Terrain Line-of-Sight (LOS) Check ---
                        % Always use the check_LOS method provided by the terrain_model object,
                        % as it correctly handles its internal data structure.
                        if terrain.data_loaded && config.terrain.ENABLE_TERRAIN_LOS
                            try
                                is_terrain_blocked = ~terrain.check_LOS(tx_lat, tx_lon, tx_alt, rx_lat, rx_lon, rx_alt);
                            catch ME_manual_los
                                warning('[RF %s->%s @ t=%.1f] Manual LOS check (terrain.check_LOS) failed: %s. Assuming blocked.', tx_id, rx_id, t_sim, ME_manual_los.message);
                                is_terrain_blocked = true; % Assume blocked if the check fails
                            end
                        else
                            is_terrain_blocked = false; % Assume clear if terrain check disabled or data not loaded
                        end

                        % --- f) Diffraction Loss (Basic placeholder if blocked) ---
                        if is_terrain_blocked
                            diffraction_loss_db = calculate_manual_diffraction_loss(is_terrain_blocked); % Add basic 20dB penalty
                        else
                            diffraction_loss_db = 0;
                        end

                        % --- g) Ground Bounce Loss (Use manual placeholder) ---
                        ground_bounce_loss_db = calculate_manual_ground_bounce_loss(range_m, tx_alt, rx_alt, lambda_m);

                    else % --- If sites_created_ok is false, fall back to MANUAL calculations ---
                         warning('[RF %s->%s @ t=%.1f] Falling back to manual calculations due to site creation failure or missing toolboxes.', tx_id, rx_id, t_sim);
                         fspl_db = calculate_manual_fspl(range_m, freq_hz);
                         gas_loss_db = calculate_manual_gas_loss(range_m, freq_hz, temperature_k, pressure_pa, rho_w);
                         if isfield(config.environment, 'rain_rate_mm_per_hr') && config.environment.rain_rate_mm_per_hr > 0
                             rain_rate = config.environment.rain_rate_mm_per_hr;
                             el_deg_ground = asind((tx_alt - rx_alt) / range_m);
                             rain_loss_db = calculate_manual_rain_loss(range_m, freq_hz, rain_rate, el_deg_ground);
                         else, rain_loss_db = 0; end
                         if isfield(config.environment, 'liquid_water_density') && config.environment.liquid_water_density > 0
                              fog_cloud_loss_db = calculate_manual_fog_loss(range_m, freq_hz);
                         else, fog_cloud_loss_db = 0; end
                         if terrain.data_loaded && config.terrain.ENABLE_TERRAIN_LOS
                             is_terrain_blocked = ~terrain.check_LOS(tx_lat, tx_lon, tx_alt, rx_lat, rx_lon, rx_alt);
                         else, is_terrain_blocked = false; end
                         diffraction_loss_db = calculate_manual_diffraction_loss(is_terrain_blocked);
                         ground_bounce_loss_db = calculate_manual_ground_bounce_loss(range_m, tx_alt, rx_alt, lambda_m);
                    end % End of if sites_created_ok block

                elseif force_manual_calc
                    % --- FORCED MANUAL CALCULATIONS (Due to earlier validation failure) ---
                     warning('[RF %s->%s @ t=%.1f] Forcing manual calculations due to invalid Tx LLH coordinates.', tx_id, rx_id, t_sim);
                     fspl_db = calculate_manual_fspl(range_m, freq_hz);
                     gas_loss_db = calculate_manual_gas_loss(range_m, freq_hz, temperature_k, pressure_pa, rho_w);
                     if isfield(config.environment, 'rain_rate_mm_per_hr') && config.environment.rain_rate_mm_per_hr > 0
                         rain_rate = config.environment.rain_rate_mm_per_hr;
                         el_deg_ground = asind((tx_alt - rx_alt) / range_m); % Note: tx_alt might be invalid here, but manual rain calc might handle it or be 0
                         rain_loss_db = calculate_manual_rain_loss(range_m, freq_hz, rain_rate, el_deg_ground);
                     else, rain_loss_db = 0; end
                     if isfield(config.environment, 'liquid_water_density') && config.environment.liquid_water_density > 0
                          fog_cloud_loss_db = calculate_manual_fog_loss(range_m, freq_hz);
                     else, fog_cloud_loss_db = 0; end
                     if terrain.data_loaded && config.terrain.ENABLE_TERRAIN_LOS && isfinite(tx_lat) && isfinite(tx_lon) && isfinite(tx_alt) % Check coords before using them
                         is_terrain_blocked = ~terrain.check_LOS(tx_lat, tx_lon, tx_alt, rx_lat, rx_lon, rx_alt);
                     else, is_terrain_blocked = false; end % Assume not blocked if coords invalid or terrain off
                     diffraction_loss_db = calculate_manual_diffraction_loss(is_terrain_blocked);
                     if isfinite(tx_alt) && isfinite(rx_alt) % Check alts before ground bounce
                         ground_bounce_loss_db = calculate_manual_ground_bounce_loss(range_m, tx_alt, rx_alt, lambda_m);
                     else, ground_bounce_loss_db = 0; end

                else % --- If no relevant toolboxes available, use MANUAL CALCULATIONS ---
                    % (This block remains the same as in v2.2)
                    %fprintf('[RF %s->%s @ t=%.1f] Using manual path.\n', tx_id, rx_id, t_sim); % Debug message
                    fspl_db = calculate_manual_fspl(range_m, freq_hz);
                    gas_loss_db = calculate_manual_gas_loss(range_m, freq_hz, temperature_k, pressure_pa, rho_w);
                    if isfield(config.environment, 'rain_rate_mm_per_hr') && config.environment.rain_rate_mm_per_hr > 0
                        rain_rate = config.environment.rain_rate_mm_per_hr;
                        el_deg_ground = asind((tx_alt - rx_alt) / range_m);
                        rain_loss_db = calculate_manual_rain_loss(range_m, freq_hz, rain_rate, el_deg_ground);
                    else, rain_loss_db = 0; end
                    if isfield(config.environment, 'liquid_water_density') && config.environment.liquid_water_density > 0
                         fog_cloud_loss_db = calculate_manual_fog_loss(range_m, freq_hz);
                    else, fog_cloud_loss_db = 0; end
                    if terrain.data_loaded && config.terrain.ENABLE_TERRAIN_LOS
                        is_terrain_blocked = ~terrain.check_LOS(tx_lat, tx_lon, tx_alt, rx_lat, rx_lon, rx_alt);
                    else, is_terrain_blocked = false; end
                    diffraction_loss_db = calculate_manual_diffraction_loss(is_terrain_blocked);
                    ground_bounce_loss_db = calculate_manual_ground_bounce_loss(range_m, tx_alt, rx_alt, lambda_m);
                end % End of conditional toolbox/manual loss calculation

                link_status.terrain_blocked = is_terrain_blocked; % Store terrain LOS result

                % --- 4.2. Aircraft Masking Check ---
                % Only perform if masking is enabled AND path is not already blocked by terrain
                if isfield(config.masking, 'ENABLE_AIRCRAFT_MASKING') && config.masking.ENABLE_AIRCRAFT_MASKING && ~is_terrain_blocked
                    masking_method = 'none'; % Default if not specified
                    if isfield(config.masking, 'METHOD'), masking_method = lower(config.masking.METHOD); end

                    if ~strcmp(masking_method, 'none')
                        try
                            % Calculate Rx position relative to aircraft ECEF center
                            rx_pos_relative_ecef = rx_pos_ecef(:) - ac_pos_ecef(:);
                            % Transform relative Rx position to aircraft body frame
                            rx_pos_body = R_ned2body * R_ecef2ned_ac * rx_pos_relative_ecef;
                            % Start point for masking check is the Tx antenna offset in body frame
                            tx_pos_body = tx_offset_body;

                            % --- Perform masking check based on method ---
                            if strcmp(masking_method, 'stl') && isfield(config.masking, 'STL_FILE')
                                stl_filepath = config.masking.STL_FILE;
                                if isfile(stl_filepath)
                                    % *** Assumes stl_LOS_intersection function exists and works ***
                                    % Check function signature: stl_LOS_intersection(start_point, end_point, stl_filepath)
                                    is_aircraft_masked = stl_LOS_intersection(tx_pos_body, rx_pos_body, stl_filepath);
                                else
                                    warning('[RF %s->%s @ t=%.1f] Masking STL file not found: %s', tx_id, rx_id, t_sim, stl_filepath);
                                    is_aircraft_masked = false; % Cannot check
                                end
                            elseif strcmp(masking_method, 'wireframe')
                                wf_points = [];
                                % Prioritize points loaded in config over file path
                                if isfield(config.masking,'WIREFRAME_POINTS') && ~isempty(config.masking.WIREFRAME_POINTS)
                                     wf_points = config.masking.WIREFRAME_POINTS;
                                elseif isfield(config.masking,'WIREFRAME_FILE') && isfile(config.masking.WIREFRAME_FILE)
                                    try % Try reading from file if points not preloaded
                                        wf_points = readmatrix(config.masking.WIREFRAME_FILE); % Assumes simple CSV format
                                    catch ME_wf_read
                                        warning('[RF %s->%s @ t=%.1f] Failed reading Wireframe file %s: %s', tx_id, rx_id, t_sim, config.masking.WIREFRAME_FILE, ME_wf_read.message);
                                    end
                                end

                                if ~isempty(wf_points)
                                     % *** Assumes wireframe_OML_masking function exists and works ***
                                     % Check signature: wireframe_OML_masking(start_point, end_point, wf_points, masking_config)
                                     is_aircraft_masked = wireframe_OML_masking(tx_pos_body, rx_pos_body, wf_points, config.masking);
                                else
                                     warning('[RF %s->%s @ t=%.1f] Wireframe masking enabled but no points/file found.', tx_id, rx_id, t_sim);
                                     is_aircraft_masked = false; % Cannot check
                                end
                            end % End wireframe check
                        catch ME_masking
                            warning('[RF %s->%s @ t=%.1f] Aircraft masking check (%s) failed: %s', tx_id, rx_id, t_sim, masking_method, ME_masking.message);
                            is_aircraft_masked = false; % Default to not masked on error
                            link_status.failure_reason = append_reason(link_status.failure_reason, 'MaskCheckErr');
                        end % End try-catch for masking calculation
                    end % End check if method is not 'none'
                end % End check if masking enabled and not terrain blocked

                link_status.aircraft_masked = is_aircraft_masked; % Store masking result

                % --- Apply LOS status based on masking result (if not already failed) ---
                if is_aircraft_masked && strcmp(link_status.status, 'OK')
                    link_status.status = 'FAIL';
                    link_status.failure_reason = append_reason(link_status.failure_reason, sprintf('AC Mask (%s)', config.masking.METHOD));
                    link_status.primary_failure_step = 'LOS - Aircraft';
                end

                % --- Store calculated losses ---
                link_status.fspl_db = fspl_db;
                link_status.gas_loss_db = gas_loss_db;
                link_status.rain_loss_db = rain_loss_db;
                link_status.fog_cloud_loss_db = fog_cloud_loss_db;
                link_status.diffraction_loss_db = diffraction_loss_db;
                link_status.ground_bounce_loss_db = ground_bounce_loss_db;

                 % --- Sum Included Losses ---
                 % Ensure all components are finite numbers before summing
                loss_components = [fspl_db, gas_loss_db, rain_loss_db, fog_cloud_loss_db, ...
                                   diffraction_loss_db, ground_bounce_loss_db];
                loss_components(~isfinite(loss_components)) = 0; % Replace NaN/Inf with 0 for summation
                total_loss_db = sum(loss_components);
                link_status.total_loss_db = total_loss_db;

                % --- 5. Received Power ---
                received_power_dbm = (tx_power_dbw + 30) + total_antenna_gain_dbi - total_loss_db;
                link_status.received_power_dbm = received_power_dbm;

                % --- 6. Noise Power ---
                % Get bandwidth, using default if not specified for the receiver
                if ~isfield(rx_cfg, 'bandwidth_hz') || isempty(rx_cfg.bandwidth_hz) || rx_cfg.bandwidth_hz <= 0
                    bandwidth_hz = config.rf.default_bandwidth_hz;
                    if i_tx==1 && i_rx==1 % Warn only once per simulation start potentially
                        warning('[RF] Rx "%s" missing/invalid BW, using default %.1f MHz.', rx_id, bandwidth_hz/1e6);
                    end
                else
                    bandwidth_hz = rx_cfg.bandwidth_hz;
                end
                % Get noise parameters from config
                noise_temp_k = config.rf.noise_temperature_k; % System noise temp
                noise_figure_db = config.rf.noise_figure_db;   % Receiver noise figure
                % Calculate thermal noise floor in dBm
                noise_power_thermal_dbm = 10*log10(K_BOLTZMANN * noise_temp_k * bandwidth_hz) + 30;
                % Add noise figure to get total effective noise power
                noise_power_dbm = noise_power_thermal_dbm + noise_figure_db;
                link_status.noise_power_dbm = noise_power_dbm;
                link_status.bandwidth_hz = bandwidth_hz;
                link_status.noise_figure_db = noise_figure_db;

                % --- 7. Signal-to-Noise Ratio (SNR) ---
                snr_db = received_power_dbm - noise_power_dbm;
                link_status.snr_db = snr_db;
                % Note: Fading loss is excluded as requested. snr_db is the effective SNR here.
                snr_db_eff = snr_db;

                % --- 8. Link Status Check vs Sensitivity ---
                rx_sensitivity_dbm = rx_cfg.sensitivity_dbm;
                link_status.rx_sensitivity_dbm = rx_sensitivity_dbm;
                % Required SNR is the gap between sensitivity and noise floor
                required_snr_db = rx_sensitivity_dbm - noise_power_dbm;
                link_status.required_snr_db = required_snr_db;
                % Check if effective SNR meets the requirement ONLY if the status is currently OK
                if snr_db_eff < required_snr_db && strcmp(link_status.status, 'OK')
                    link_status.status = 'FAIL';
                    link_status.failure_reason = append_reason(link_status.failure_reason, sprintf('SNR < Req (%.1f<%.1f)', snr_db_eff, required_snr_db));
                    link_status.primary_failure_step = 'Signal Level';
                end

                % --- 9. Adaptive Modulation & Data Rate ---
                % Initialize outputs
                link_status.modulation_state = 'N/A';
                link_status.data_rate_bps = 0;
                % Proceed only if modulation is enabled and schemes are defined
                if isfield(config.rf, 'ENABLE_ADAPTIVE_MODULATION') && config.rf.ENABLE_ADAPTIVE_MODULATION && isfield(config, 'modulation') && ~isempty(config.modulation.schemes)
                    try
                        selected_scheme_name = 'N/A';
                        selected_rate_bps = 0;
                        % Iterate through defined schemes (assuming sorted high to low rate in config)
                        for i_mod = 1:length(config.modulation.schemes)
                            scheme = config.modulation.schemes{i_mod};
                            % Check if effective SNR meets the minimum required for this scheme
                            if isfield(scheme, 'min_snr_db') && snr_db_eff >= scheme.min_snr_db
                                selected_scheme_name = scheme.name;
                                if isfield(scheme, 'data_rate_mbps')
                                    selected_rate_bps = scheme.data_rate_mbps * 1e6;
                                end
                                break; % Found the highest rate scheme that works
                            end
                        end
                        link_status.modulation_state = selected_scheme_name;
                        link_status.data_rate_bps = selected_rate_bps;

                        % If no valid modulation could be selected and status was OK, mark as FAIL
                        if strcmp(selected_scheme_name, 'N/A') && strcmp(link_status.status, 'OK')
                            link_status.status = 'FAIL';
                            link_status.failure_reason=append_reason(link_status.failure_reason, 'No Valid Mod');
                            link_status.primary_failure_step='Modulation';
                        end
                    catch ME_mod
                        warning('[RF %s->%s @ t=%.1f] Adaptive Modulation failed: %s', tx_id, rx_id, t_sim, ME_mod.message);
                         if strcmp(link_status.status, 'OK') % Fail if calculation error prevented selection
                             link_status.status = 'FAIL';
                             link_status.failure_reason=append_reason(link_status.failure_reason, 'Mod Calc Err');
                             link_status.primary_failure_step='Modulation';
                         end
                    end
                end % End Adaptive Modulation block

                % --- 10. BER / SER Estimation ---
                link_status.ber = NaN; % Initialize
                % Proceed only if enabled, modulation was selected, and curves exist
                if isfield(config.rf, 'ENABLE_BER_SER_ESTIMATION') && config.rf.ENABLE_BER_SER_ESTIMATION && ...
                   ~strcmp(link_status.modulation_state, 'N/A') && isfield(config, 'modulation') && isfield(config.modulation, 'schemes')

                    try
                        % Find the BER curve for the selected modulation state
                        selected_mod_idx = find(strcmp({config.modulation.schemes.name}, link_status.modulation_state), 1);
                        if ~isempty(selected_mod_idx) && isfield(config.modulation.schemes{selected_mod_idx}, 'ber_snr_curve')
                             curve = config.modulation.schemes{selected_mod_idx}.ber_snr_curve; % Should be Nx2 array [SNR_dB, BER]
                             if size(curve, 2) == 2 && size(curve, 1) > 1
                                 snr_points_curve = curve(:, 1);
                                 ber_points_curve = curve(:, 2);
                                 % Ensure BER values are positive for log interpolation
                                 ber_points_curve(ber_points_curve <= 0) = 1e-12; % Prevent log(0)

                                 % Interpolate in log-BER domain for better accuracy
                                 if snr_db_eff < snr_points_curve(1)
                                     link_status.ber = ber_points_curve(1); % Clamp to min SNR BER
                                 elseif snr_db_eff > snr_points_curve(end)
                                     link_status.ber = ber_points_curve(end); % Clamp to max SNR BER
                                 else
                                     log_ber = interp1(snr_points_curve, log10(ber_points_curve), snr_db_eff, 'linear');
                                     link_status.ber = 10^log_ber;
                                 end
                                 % Simple SER estimation (e.g., SER = M*BER for M-PSK/M-QAM approx) - requires knowing M
                                 % link_status.ser = link_status.ber * get_modulation_order(link_status.modulation_state);
                             else
                                 warning('[RF %s->%s @ t=%.1f] BER curve format invalid for %s.', tx_id, rx_id, t_sim, link_status.modulation_state);
                             end
                        else
                             warning('[RF %s->%s @ t=%.1f] BER curve not found for %s.', tx_id, rx_id, t_sim, link_status.modulation_state);
                        end
                    catch ME_ber
                         warning('[RF %s->%s @ t=%.1f] BER Estimation failed: %s', tx_id, rx_id, t_sim, ME_ber.message);
                    end
                end % End BER Estimation block

                % --- 11. Link Margin ---
                link_status.link_margin_db = received_power_dbm - rx_sensitivity_dbm;

                % --- 12. Doppler Shift (Basic LOS component) ---
                doppler_hz = 0; % Initialize
                if isfield(config.rf, 'ENABLE_DOPPLER_SHIFT') && config.rf.ENABLE_DOPPLER_SHIFT
                    try
                        rx_vel_ecef = [0; 0; 0]; % Assume stationary ground receiver
                        relative_velocity_ecef = ac_vel_ecef - rx_vel_ecef;
                        % Project relative velocity onto the LOS unit vector
                        relative_speed_los = dot(relative_velocity_ecef, los_unit_ecef); % Positive if moving away
                        % Calculate Doppler shift (negative for closing speed)
                        doppler_hz = -(relative_speed_los / C_LIGHT) * freq_hz;
                    catch ME_dop
                        warning('[RF %s->%s @ t=%.1f] Doppler Shift calculation failed: %s', tx_id, rx_id, t_sim, ME_dop.message);
                    end
                end
                link_status.doppler_shift_hz = doppler_hz;
                % Note: Doppler Spread calculation excluded as requested.

                % --- 13. Shannon Capacity (Theoretical Limit) ---
                capacity_bps = NaN; % Initialize
                if isfield(config.rf, 'ENABLE_SHANNON_CAPACITY') && config.rf.ENABLE_SHANNON_CAPACITY && bandwidth_hz > 0 && isfinite(snr_db)
                    snr_linear = 10^(snr_db / 10);
                    if snr_linear < 0, snr_linear = 0; end % Ensure non-negative SNR linear
                    capacity_bps = bandwidth_hz * log2(1 + snr_linear);
                end
                link_status.shannon_capacity_bps = capacity_bps;

            catch ME_link_calc  % Catch errors during calculation for a specific link
                warning('[RF MODEL %s->%s @ t=%.1f] Error during link calculation: %s. Marking link FAIL.', tx_id, rx_id, t_sim, ME_link_calc.message);
                % Ensure basic info is populated even if error occurred mid-calculation
                link_status.tx_id = tx_id;
                link_status.rx_id = rx_id;
                link_status.time_s = t_sim;
                link_status.status = 'FAIL';
                link_status.failure_reason = append_reason(link_status.failure_reason, 'CalcError');
                link_status.primary_failure_step = 'Calculation Error';
                % Populate remaining fields with NaN or defaults to ensure structure consistency
                fields_to_reset = fieldnames(link_status);
                for f_idx = 1:length(fields_to_reset)
                    fname = fields_to_reset{f_idx};
                    current_val = link_status.(fname);
                    if isempty(current_val) || (isnumeric(current_val) && (isnan(current_val) || isinf(current_val))) % Check empty, NaN, Inf
                        % Get default value from template
                        template = create_link_status_struct_helper(); % UPDATED HELPER CALL
                        link_status.(fname) = template.(fname);
                    end
                end
            end % End try-catch for individual link calculation

            % --- Store the final status structure for this link ---
            % This assignment should now be safe from "dissimilar structures" error
            % because link_status was initialized from the helper and all fields
            % should be populated (even with defaults/NaNs in the catch block).
             rf_status(link_idx) = link_status;

        end % End Rx loop
    end % End Tx loop

    % --- Aggregate Failure Summary ---
    try
        valid_links_mask = ~strcmp({rf_status.status}, 'SKIP') & ~strcmp({rf_status.status}, 'PENDING');
        if ~any(valid_links_mask)
            failure_summary.counts = struct();
            failure_summary.total_calculated_links = 0;
            failure_summary.total_failed_links = 0;
        else
            failed_links_mask = strcmp({rf_status(valid_links_mask).status}, 'FAIL');
            failure_steps_list = {rf_status(valid_links_mask).primary_failure_step};
            failure_steps_list = failure_steps_list(failed_links_mask); % Get steps only for failed links

            unique_steps = unique(failure_steps_list);
            failure_summary.counts = struct();
            failure_summary.total_calculated_links = sum(valid_links_mask);
            failure_summary.total_failed_links = sum(failed_links_mask);
            for k = 1:length(unique_steps)
                step_name = unique_steps{k};
                if isempty(step_name), continue; end % Skip empty step names
                count = sum(strcmp(failure_steps_list, step_name));
                field_name = matlab.lang.makeValidName(step_name); % Ensure valid field name
                failure_summary.counts.(field_name) = count;
            end
        end
    catch ME_summary
        warning('[RF MODEL @ t=%.1f] Failed to generate failure summary: %s', t_sim, ME_summary.message);
        failure_summary.error = ME_summary.message;
    end

end % END compute_rf_links_internal

% =========================================================================
% RF HELPER FUNCTIONS (Manual Calculations & Struct Definition)
% =========================================================================

% --- CREATE LINK STATUS STRUCT HELPER ---
function link_status = create_link_status_struct_helper()
    % Creates an empty structure with all fields pre-defined for this version (v2.1).
    % Ensures consistency and helps prevent "dissimilar structures" errors.
    link_status = struct( ...
        'time_s', NaN, 'tx_id', '', 'rx_id', '', ...           % Identifiers
        'status', 'PENDING', 'failure_reason', '', 'primary_failure_step', '', ... % Status
        'frequency_hz', NaN, 'range_m', NaN, ...               % Basic Geometry
        'azimuth_body_deg', NaN, 'elevation_body_deg', NaN, ...% Angles (Tx Body Frame)
        'terrain_blocked', false, 'aircraft_masked', false, ...% LOS Checks (Added aircraft_masked)
        'tx_power_dbm', NaN, 'tx_gain_dbi', NaN, 'rx_gain_dbi', NaN, ... % Gains/Power
        ... % --- Included Losses (dB) ---
        'fspl_db', NaN, ...                   % Free Space Path Loss
        'gas_loss_db', 0, ...                 % Atmospheric Gas Loss
        'rain_loss_db', 0, ...                % Rain Loss
        'fog_cloud_loss_db', 0, ...           % Fog / Cloud Loss
        'diffraction_loss_db', 0, ...         % Terrain Diffraction (basic)
        'ground_bounce_loss_db', 0, ...       % Ground Bounce (basic)
        'total_loss_db', NaN, ...             % Sum of included losses
        ... % --- Link Budget Results ---
        'received_power_dbm', NaN, ...        % Pr
        'noise_figure_db', NaN, 'bandwidth_hz', NaN, 'noise_power_dbm', NaN, ... % Noise
        'rx_sensitivity_dbm', NaN, 'required_snr_db', NaN, ... % Thresholds
        'snr_db', NaN, ...                    % SNR (average, before potential fading)
        'link_margin_db', NaN, ...            % Margin above sensitivity
        ... % --- Performance Metrics ---
        'modulation_state', 'N/A', ...        % Selected modulation (string)
        'data_rate_bps', 0, ...               % Achieved data rate (bps)
        'ber', NaN, ...                       % Bit Error Rate estimate
        'shannon_capacity_bps', NaN, ...      % Theoretical limit
        'doppler_shift_hz', NaN ...           % Base Doppler shift
    );
end

% --- MANUAL FSPL CALCULATION ---
function fspl_db = calculate_manual_fspl(range_m, freq_hz)
    % Calculates Free Space Path Loss using the standard formula.
    if range_m <= 0 || freq_hz <= 0
        fspl_db = Inf; % Avoid log(0) or nonsensical results
        return;
    end
    fspl_db = 20*log10(range_m) + 20*log10(freq_hz) - 147.55;  % dB
end

% --- MANUAL GAS LOSS CALCULATION (Simplified ITU P.676 Approx) ---
function gas_loss_db = calculate_manual_gas_loss(range_m, freq_hz, temp_k, pressure_pa, rho_w)
    % Simple approximation for gaseous attenuation based on ITU-R P.676 concepts.
    % Note: This is a very basic placeholder and less accurate than gaspl.
    try
        freq_GHz = freq_hz / 1e9;
        pressure_hPa = pressure_pa / 100; % Convert Pa to hPa
        if freq_GHz <= 0, gas_loss_db = 0; return; end

        % Simplified formulas (vary significantly with frequency and conditions)
        % Oxygen approx - placeholder values
        gamma_o = (0.007 + 6 / ( (freq_GHz - 60)^2 + 1) ) * (pressure_hPa/1013.25) * (293/temp_k)^2;
        % Water vapor approx - placeholder values
        gamma_w = (0.05 + 0.002*rho_w + 4 / ((freq_GHz - 22.2)^2 + 2)) * rho_w * (293/temp_k)^1.5;

        total_specific_attenuation_db_per_km = gamma_o + gamma_w; % [dB/km]
        path_km = range_m / 1000;
        gas_loss_db = total_specific_attenuation_db_per_km * path_km;
        gas_loss_db = max(0, gas_loss_db); % Ensure non-negative loss
    catch
        gas_loss_db = 0; % Default to zero loss on error
    end
end

% --- MANUAL RAIN LOSS CALCULATION (Simplified ITU P.838 Approx) ---
function rain_loss_db = calculate_manual_rain_loss(range_m, freq_hz, rain_rate, el_deg)
    % Simple approximation for rain attenuation based on ITU-R P.838 concepts.
    % Note: Less accurate than rainpl.
    try
        if rain_rate <= 0, rain_loss_db = 0; return; end
        freq_GHz = freq_hz / 1e9;
        el_deg = max(el_deg, 0.1); % Avoid division by zero

        % Get k and alpha parameters (very rough fit examples)
        if freq_GHz < 6
            k = 4e-5 * freq_GHz^2.4; alpha = 0.9 + 0.01*freq_GHz;
        else
            k = 4e-3 * freq_GHz^0.7; alpha = 1.1 - 0.005*freq_GHz;
        end
        k = max(k, 1e-6); alpha = max(alpha, 0.5);

        % Horizontal attenuation (dB/km)
        gamma_r = k * (rain_rate)^alpha;
        % Effective path length (simplified)
        path_km = range_m / 1000;
        Leff_km = path_km / sind(el_deg); % Very simple slant path
        Leff_km = min(Leff_km, 50); % Cap effective length

        rain_loss_db = gamma_r * Leff_km;
        rain_loss_db = max(0, rain_loss_db); % Ensure non-negative loss
    catch
         rain_loss_db = 0; % Default to zero loss on error
    end
end

% --- MANUAL FOG/CLOUD LOSS CALCULATION (Basic Placeholder) ---
function fog_loss_db = calculate_manual_fog_loss(range_m, freq_hz)
    % Extremely basic placeholder for fog/cloud loss.
    try
        freq_GHz = freq_hz / 1e9;
        % Assume a simple specific attenuation like 0.01 dB/km/GHz
        fog_specific_loss_db_per_km = 0.01 * freq_GHz;
        path_km = range_m / 1000;
        fog_loss_db = fog_specific_loss_db_per_km * path_km;
        fog_loss_db = max(0, fog_loss_db);
    catch
        fog_loss_db = 0;
    end
end

% --- MANUAL DIFFRACTION LOSS CALCULATION (Basic Placeholder) ---
function diff_loss_db = calculate_manual_diffraction_loss(is_blocked)
    % Adds a fixed penalty if the path is known to be blocked by terrain.
    if is_blocked
        diff_loss_db = 20.0; % Typical minimum diffraction loss over an obstacle
    else
        diff_loss_db = 0.0;
    end
end

% --- MANUAL GROUND BOUNCE LOSS CALCULATION (Simple 2-Ray Placeholder) ---
function gb_loss_db = calculate_manual_ground_bounce_loss(range_m, h_tx, h_rx, lambda_m)
    % Simple 2-ray model placeholder. Assumes flat earth locally for geometry.
    % Ignores actual terrain profile and reflection coefficients.
    try
        if range_m <= 0 || lambda_m <= 0 || h_tx <= 0 || h_rx <= 0
            gb_loss_db = 0; % Not applicable or invalid inputs
            return;
        end
        % Path length difference approx for 2-ray model over flat earth
        delta_path = (2 * h_tx * h_rx) / range_m;
        % Phase difference due to path length difference
        delta_phase = (2 * pi * delta_path) / lambda_m;
        % Assume a perfect reflection coefficient magnitude of 1 and phase shift of pi (-1)
        R_ground = -1.0;
        % Calculate magnitude relative to direct path |1 + R*exp(-j*delta_phase)|
        magnitude_factor = abs(1 + R_ground * exp(-1i * delta_phase));
        % Convert magnitude factor change to dB loss (can be gain or loss)
        % Loss is -20*log10(magnitude_factor). We report loss as positive dB.
        % Note: This model can predict gain (negative loss) which might not be realistic.
        % Cap the loss calculation to be non-negative for simplicity as a loss term.
        gb_loss_db = max(0, -20 * log10(magnitude_factor));
        % Add a sanity cap (e.g., max 20 dB loss from this effect)
        gb_loss_db = min(gb_loss_db, 20.0);

    catch
        gb_loss_db = 0; % Default to zero loss on error
    end
end


% --- ANGLE TO DCM HELPER ---
function R_body2ned = angle2dcm_local(yaw, pitch, roll, order)
    % Calculates Direction Cosine Matrix from Euler Angles (ZYX order assumed needed).
    % Input angles are in RADIANS.
    cy=cos(yaw); sy=sin(yaw);
    cp=cos(pitch); sp=sin(pitch);
    cr=cos(roll); sr=sin(roll);
    if strcmpi(order, 'ZYX') % Yaw(Z), Pitch(Y), Roll(X) rotation sequence
        R_body2ned=[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr;
                    sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr;
                      -sp,           cp*sr,           cp*cr];
    else
        error('Rotation order %s not implemented in angle2dcm_local.', order);
    end
end

% --- APPEND REASON HELPER ---
function reason_out = append_reason(reason_in, new_msg)
    % Safely appends a new message to the failure reason string, avoiding duplication.
    if isempty(reason_in)
        reason_out = new_msg;
    elseif contains(reason_in, new_msg) % Don't add if already present
        reason_out = reason_in;
    else
        reason_out = [reason_in, '; ', new_msg];
    end
end
