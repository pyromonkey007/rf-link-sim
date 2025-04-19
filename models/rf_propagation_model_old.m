% =========================================================================
% RF PROPAGATION MODEL (FINAL - Re-Commented)
% Version: FINAL 1.0
%
% Description:
%   Calculates the detailed RF link budget and performance metrics between
%   each *supported* aircraft transmitter and ground station receiver pair
%   at a given simulation time step. This is typically called post-simulation
%   using the final trajectory data.
%
% Core Functionality:
%   - Iterates through all Tx-Rx pairs defined in the configuration.
%   - **Band Filtering:** Checks if the receiver supports the transmitter's ID. Skips if not.
%   - Geometry Calculation: Calculates Line-of-Sight (LOS) vector and range (using earth model).
%   - Coordinate Transforms: Transforms LOS vector to Body frame for Az/El calculation.
%   - Antenna Gain Lookup: Gets Tx gain based on body-relative Az/El angles (uses antenna model).
%   - LOS Checks: Calls terrain model and aircraft masking functions.
%   - Link Budget Calculation:
%       - Free Space Path Loss (FSPL)
%       - Atmospheric Loss (**Placeholder - Requires Implementation**)
%       - Rain Loss (**Placeholder - Requires Implementation**)
%       - Polarization Mismatch Loss (from config)
%       - Multipath Loss/Delay (**Placeholder - Requires Implementation**)
%       - Other Losses (e.g., Fresnel - **Placeholder**)
%   - Doppler Shift: Calculates based on relative velocity between AC and GS (using earth model transforms).
%   - Received Power (Pr): Calculates based on Tx power, gains, and total losses.
%   - Noise Power: Calculates based on receiver bandwidth, noise figure, and temperature (from config).
%   - SNR: Calculates Signal-to-Noise Ratio.
%   - Shannon Capacity: Calculates theoretical maximum data rate.
%   - Status Determination: Sets link status ('OK', 'FAIL', 'SKIP') and logs primary failure reason/step.
%
% Inputs (to internal compute_rf function):
%   config     - The main configuration structure.
%   state      - Current aircraft state {lat_deg, lon_deg, alt_m_msl, heading_deg, pitch_deg, roll_deg, velocity_ned_mps}.
%   earth      - Initialized earth model object.
%   atmosphere - Initialized atmospheric model object.
%   terrain    - Initialized terrain model object.
%   antennas   - Initialized antenna models structure.
%   t_sim      - Current simulation time (seconds).
%
% Outputs (from internal compute_rf function):
%   rf_status       - Structure array (size = num_tx * num_rx). Each element contains
%                     detailed results for one potential link (SNR, capacity, losses, status, reason).
%                     Status will be 'SKIP' for unsupported Tx/Rx pairs.
%   failure_summary - Structure summarizing primary failure reasons across *calculated* (non-skipped) links.
%
% Placeholders:
%   - Atmospheric Loss calculation needs proper implementation (e.g., ITU P.676).
%   - Rain Loss calculation needs implementation (e.g., ITU P.838, requires rain rate data).
%   - Multipath Effects calculation needs implementation (statistical or physics-based).
%   - Fresnel Zone check needs implementation.
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-04]
% =========================================================================
function rf_model_obj = rf_propagation_model()
    % Constructor for the RF model. Returns an object with a handle
    % to the internal calculation function.
    rf_model_obj.compute_rf = @compute_rf_links_internal;
    fprintf('[RF MODEL] RF Propagation Model Initialized.\n');
end % END OF CONSTRUCTOR

% =========================================================================
% INTERNAL FUNCTION: COMPUTE RF LINKS FOR ALL TX/RX PAIRS
% =========================================================================
function [rf_status, failure_summary] = compute_rf_links_internal(config, state, earth, atmosphere, terrain, antennas, t_sim)
    % This function performs the core calculations for all Tx-Rx pairs at a single time step.

    % --- Constants ---
    C_LIGHT     = 299792458;      % Speed of light (m/s)
    K_BOLTZMANN = 1.380649e-23;   % Boltzmann constant (J/K)
    DEG2RAD     = pi/180;
    RAD2DEG     = 180/pi;

    % --- Basic Checks ---
    % Check if RF module is enabled globally
    if ~config.rf.ENABLE_RF_MODULE
        rf_status = struct('status', 'RF Disabled'); % Minimal status
        failure_summary = struct('reason', 'RF Disabled');
        return;
    end
    % Check for required state information (velocity for Doppler)
    if ~isfield(state, 'velocity_ned_mps') || isempty(state.velocity_ned_mps) || length(state.velocity_ned_mps) ~= 3
        error('RF Model requires state structure to include velocity_ned_mps [N; E; D] vector.');
    end
    % Check if transmitters and receivers are defined
    if isempty(config.rf.transmitters) || isempty(config.rf.ground_stations)
         warning('[RF MODEL] No transmitters or ground stations defined in config. Skipping RF calculations.');
         rf_status = struct('status', 'No Tx/Rx defined');
         failure_summary = struct('reason', 'No Tx/Rx defined');
         return;
    end

    % --- Initialize Output Structures ---
    num_tx    = length(config.rf.transmitters);
    num_rx    = length(config.rf.ground_stations);
    num_links = num_tx * num_rx; % Total number of potential links
    % Preallocate structure array for results (one element per potential link)
    rf_status = repmat(create_link_status_struct_helper(), num_links, 1);
    link_idx  = 0; % Master index for the rf_status array
    failure_summary = struct(); % Will summarize failures at the end

    % --- Calculate Aircraft ECEF Position and Velocity (once per step) ---
    % Needed for range and relative velocity calculations.
    try
        [ac_pos_ecef(1), ac_pos_ecef(2), ac_pos_ecef(3)] = earth.LLH_to_ECEF(state.lat_deg, state.lon_deg, state.alt_m_msl);
        % Get rotation matrix from local NED (at aircraft location) to ECEF
        R_ned2ecef = earth.NED_to_ECEF_matrix(state.lat_deg, state.lon_deg);
        % Transform aircraft velocity vector from NED to ECEF frame
        ac_vel_ecef = R_ned2ecef * state.velocity_ned_mps(:); % Ensure velocity is column vector [3x1]
    catch ME_ecef
        error('Failed to calculate aircraft ECEF state: %s', ME_ecef.message);
    end

    % =====================================================================
    % === Loop Through Each Transmitter ===
    % =====================================================================
    for i_tx = 1:num_tx
        % --- Get Transmitter Configuration ---
        tx_cfg        = config.rf.transmitters{i_tx};
        tx_id         = tx_cfg.id;                     % Unique ID string
        tx_field_name = matlab.lang.makeValidName(tx_id); % ID for struct field access

        % --- Check if Antenna Model Exists ---
        if ~isfield(antennas, tx_field_name)
            warning('[RF MODEL] Antenna model for transmitter "%s" not found. Skipping this transmitter.', tx_id);
            % Populate status as 'SKIP' for all receivers for this missing Tx
            for i_rx_skip = 1:num_rx
                link_idx = link_idx + 1;
                rx_cfg_skip = config.rf.ground_stations{i_rx_skip};
                rf_status(link_idx).time_s = t_sim; rf_status(link_idx).tx_id = tx_id; rf_status(link_idx).rx_id = rx_cfg_skip.id;
                rf_status(link_idx).status = 'SKIP'; rf_status(link_idx).failure_reason = 'Antenna Model Missing'; rf_status(link_idx).primary_failure_step = 'Setup';
            end
            continue; % Go to the next transmitter
        end
        ant_model      = antennas.(tx_field_name); % Get antenna object/struct
        freq_hz        = tx_cfg.freq_hz;           % Frequency (Hz)
        tx_power_dbw   = tx_cfg.power_dbw;         % Tx Power (dBW)
        tx_offset_body = tx_cfg.offset_body_m(:);  % Tx Antenna offset in Body frame [X; Y; Z] (m)

        % --- Calculate Tx Antenna Absolute Position in ECEF ---
        % Rotate offset from Body frame to NED frame, then to ECEF frame.
        try
            % Rotation from Body frame (X-fwd, Y-right, Z-down) to NED frame
            R_body2ned = angle2dcm_local(state.heading_deg*DEG2RAD, state.pitch_deg*DEG2RAD, state.roll_deg*DEG2RAD, 'ZYX');
            % Transform offset vector and add to aircraft CG ECEF position
            tx_offset_ecef = R_ned2ecef * R_body2ned * tx_offset_body;
            tx_pos_ecef    = ac_pos_ecef(:) + tx_offset_ecef; % Antenna ECEF position [3x1]
        catch ME_tx_pos
             warning('[RF MODEL Tx=%s] Failed to calculate Tx antenna ECEF position: %s. Skipping Tx.', tx_id, ME_tx_pos.message);
             % Populate status as 'SKIP' for remaining receivers for this Tx
             remaining_rx = num_rx - (link_idx - (i_tx-1)*num_rx); % Num Rx left for this Tx
             for i_rx_skip = 1:remaining_rx
                 link_idx = link_idx + 1; rx_cfg_skip = config.rf.ground_stations{i_rx_skip + (link_idx - (i_tx-1)*num_rx - remaining_rx)}; % Get correct Rx cfg
                 rf_status(link_idx).time_s = t_sim; rf_status(link_idx).tx_id = tx_id; rf_status(link_idx).rx_id = rx_cfg_skip.id;
                 rf_status(link_idx).status = 'SKIP'; rf_status(link_idx).failure_reason = 'Tx Position Error'; rf_status(link_idx).primary_failure_step = 'Setup';
             end
             continue; % Skip to next transmitter
        end

        % =====================================================================
        % === Loop Through Each Ground Station (Receiver) ===
        % =====================================================================
        for i_rx = 1:num_rx
            link_idx = link_idx + 1; % Increment master index for rf_status array

            % --- Get Receiver Configuration ---
            rx_cfg = config.rf.ground_stations{i_rx};
            rx_id  = rx_cfg.id;

            % --- Initialize Status Struct for This Link ---
            % Use helper to ensure all fields exist.
            link_status                     = rf_status(link_idx); % Get handle to preallocated struct
            link_status.time_s              = t_sim;
            link_status.tx_id               = tx_id;
            link_status.rx_id               = rx_id;
            link_status.frequency_hz        = freq_hz;
            link_status.tx_power_dbm        = tx_power_dbw + 30; % Convert dBW to dBm
            link_status.status              = 'PENDING'; % Start as pending validation
            link_status.failure_reason      = ''; % Clear any previous reason
            link_status.primary_failure_step= ''; % Clear any previous step

            % --- Band Filtering Check ---
            % Does this receiver support the current transmitter ID?
            supports_tx = false;
            if isfield(rx_cfg, 'supported_tx_ids') && iscellstr(rx_cfg.supported_tx_ids)
                 if ismember(tx_id, rx_cfg.supported_tx_ids)
                     supports_tx = true;
                 end
            end

            if ~supports_tx
                % Mark this link as skipped and move to the next receiver.
                link_status.status = 'SKIP';
                link_status.failure_reason = 'Rx does not support this Tx band/ID';
                link_status.primary_failure_step = 'Setup';
                rf_status(link_idx) = link_status; % Store skipped status
                continue; % <<< SKIP TO NEXT RECEIVER >>>
            end

            % --- If supported, proceed with link budget calculation ---
            link_status.status = 'OK'; % Assume OK initially

            try % Wrap calculations in try-catch for robustness per link

                % --- Get Receiver Position in ECEF ---
                [rx_pos_ecef(1), rx_pos_ecef(2), rx_pos_ecef(3)] = earth.LLH_to_ECEF(rx_cfg.lat_deg, rx_cfg.lon_deg, rx_cfg.alt_m_msl);

                % --- 1. Calculate Geometry (LOS Vector, Range, Angles) ---
                los_vector_ecef = rx_pos_ecef(:) - tx_pos_ecef(:); % Vector from Tx antenna to Rx antenna [3x1]
                range_m         = norm(los_vector_ecef);          % Slant range (m)
                link_status.range_m = range_m;

                if range_m < 1.0 % Check for near-zero range
                    link_status.status = 'FAIL'; link_status.failure_reason = 'Range < 1m'; link_status.primary_failure_step = 'Geometry';
                    rf_status(link_idx) = link_status; continue; % Skip rest of calcs for this link
                end
                los_unit_ecef = los_vector_ecef / range_m; % Normalized LOS vector in ECEF [3x1]

                % --- 2. Coordinate Transformations for Angles and Masking ---
                % Transform LOS vector from ECEF to Aircraft NED frame (origin at AC CG)
                R_ecef2ned_ac = earth.ECEF_to_NED_matrix(state.lat_deg, state.lon_deg);
                los_vector_ned = R_ecef2ned_ac * los_vector_ecef;
                % Transform LOS vector from NED to Aircraft Body frame (X-fwd, Y-right, Z-down)
                R_ned2body = R_body2ned'; % Body->NED transpose is NED->Body
                los_vector_body = R_ned2body * los_vector_ned; % [X_b; Y_b; Z_b]

                % Calculate Azimuth/Elevation relative to BODY frame boresight (X-axis)
                % Azimuth (Look Angle): Angle in XY plane from X-axis. [0, 360) deg, +ve right.
                az_body_deg = mod(atan2d(los_vector_body(2), los_vector_body(1)), 360);
                % Elevation (Depression Angle): Angle from XY plane. [-90, +90] deg, +ve Z (Down).
                el_body_deg = atan2d(los_vector_body(3), sqrt(los_vector_body(1)^2 + los_vector_body(2)^2));

                link_status.azimuth_body_deg   = az_body_deg;
                link_status.elevation_body_deg = el_body_deg;
                % Note: Assumes antenna boresight aligns with body X-axis. Adjust az/el here
                % if antenna has a mounting offset angle relative to body frame.

                % --- 3. Antenna Gain Lookup ---
                tx_gain_dbi = ant_model.get_gain(az_body_deg, el_body_deg); % Pass body-relative angles
                rx_gain_dbi = rx_cfg.rx_gain_dbi; % Get Rx gain from config (assumed fixed)
                total_antenna_gain_dbi = tx_gain_dbi + rx_gain_dbi;
                link_status.tx_gain_dbi = tx_gain_dbi;
                link_status.rx_gain_dbi = rx_gain_dbi;

                % --- 4. Terrain LOS Check ---
                is_terrain_blocked = false;
                if config.terrain.ENABLE_TERRAIN_LOS && terrain.data_loaded % Check terrain enabled & loaded
                    try
                        % Need Tx antenna position in LLH for terrain check
                        [tx_pos_llh(1), tx_pos_llh(2), tx_pos_llh(3)] = earth.ECEF_to_LLH(tx_pos_ecef(1), tx_pos_ecef(2), tx_pos_ecef(3));
                        % Call terrain model's LOS check function
                        is_terrain_blocked = ~terrain.check_LOS(tx_pos_llh(1), tx_pos_llh(2), tx_pos_llh(3), ...
                                                               rx_cfg.lat_deg, rx_cfg.lon_deg, rx_cfg.alt_m_msl);
                    catch ME_terrain_los
                         warning('[RF MODEL %s->%s] Terrain LOS check failed: %s', tx_id, rx_id, ME_terrain_los.message);
                         is_terrain_blocked = true; % Assume blocked on error for safety
                         link_status.failure_reason = append_reason(link_status.failure_reason, 'Terrain Check Error');
                    end
                end
                link_status.terrain_blocked = is_terrain_blocked;
                if is_terrain_blocked % If blocked, set status to FAIL but continue calculating losses for logging
                    link_status.status = 'FAIL';
                    link_status.failure_reason = append_reason(link_status.failure_reason, 'Terrain Blockage');
                    link_status.primary_failure_step = 'LOS - Terrain';
                end

                % --- 5. Aircraft Masking Check (Patch 10) ---
                is_aircraft_masked = false;
                % Only perform check if enabled AND path is not already blocked by terrain
                if config.masking.ENABLE_AIRCRAFT_MASKING && ~is_terrain_blocked
                    masking_method = config.masking.METHOD;
                    try
                        % Need Tx and Rx positions relative to Aircraft CG in the BODY frame
                        % Tx position relative to CG is tx_offset_body [X; Y; Z]
                        tx_pos_mask = tx_offset_body;

                        % Rx position relative to CG in BODY frame:
                        % Vector from AC CG to Rx in ECEF
                        rx_pos_relative_ecef = rx_pos_ecef(:) - ac_pos_ecef(:);
                        % Transform this vector to AC Body Frame [X; Y; Z]
                        rx_pos_mask = R_ned2body * R_ecef2ned_ac * rx_pos_relative_ecef;

                        % Call appropriate masking function based on method
                        if strcmpi(masking_method, 'stl')
                            stl_filepath = config.masking.STL_FILE;
                            if isfile(stl_filepath)
                                is_aircraft_masked = stl_LOS_intersection(tx_pos_mask, rx_pos_mask, stl_filepath);
                            else
                                 warning('[RF MODEL %s->%s] Aircraft masking STL file not found: %s. Skipping check.', tx_id, rx_id, stl_filepath);
                            end
                        elseif strcmpi(masking_method, 'wireframe')
                            wf_filepath = config.masking.WIREFRAME_FILE;
                            if isfield(config.masking,'WIREFRAME_POINTS') % Use points from config if preloaded
                                 wf_points = config.masking.WIREFRAME_POINTS;
                                 if ~isempty(wf_points)
                                      is_aircraft_masked = wireframe_OML_masking(tx_pos_mask, rx_pos_mask, wf_points, config.masking);
                                 else
                                     warning('[RF MODEL %s->%s] Wireframe points empty in config. Skipping check.', tx_id, rx_id);
                                 end
                            elseif isfile(wf_filepath) % Load from file if points not in config
                                 try
                                     wf_points = readmatrix(wf_filepath); % Assumes simple X,Y csv
                                     is_aircraft_masked = wireframe_OML_masking(tx_pos_mask, rx_pos_mask, wf_points, config.masking);
                                 catch ME_wf_read
                                      warning('[RF MODEL %s->%s] Failed to read wireframe file %s: %s. Skipping check.', tx_id, rx_id, wf_filepath, ME_wf_read.message);
                                 end
                            else
                                 warning('[RF MODEL %s->%s] Aircraft masking Wireframe file not specified/found. Skipping check.', tx_id, rx_id);
                            end
                        end % End masking method check
                    catch ME_masking
                        warning('[RF MODEL %s->%s] Aircraft masking check failed (%s): %s', tx_id, rx_id, masking_method, ME_masking.message);
                        is_aircraft_masked = false; % Assume not masked on error?
                        link_status.failure_reason = append_reason(link_status.failure_reason, 'Aircraft Masking Error');
                    end
                end % End masking enabled check
                link_status.aircraft_masked = is_aircraft_masked;
                if is_aircraft_masked && strcmp(link_status.status, 'OK') % Set fail only if not already failed
                    link_status.status = 'FAIL';
                    link_status.failure_reason = append_reason(link_status.failure_reason, sprintf('Aircraft Masking (%s)', config.masking.METHOD));
                    link_status.primary_failure_step = 'LOS - Aircraft';
                    % Continue calculating losses for logging
                end

                % --- 6. Free Space Path Loss (FSPL) ---
                fspl_db = 20*log10(range_m) + 20*log10(freq_hz) - 147.55; % dB (positive value)
                link_status.fspl_db = fspl_db;

                % --- 7. Atmospheric Loss ---
                % ** Placeholder Call - Replace with proper implementation **
                atm_loss_db = calculate_atmospheric_loss_placeholder(freq_hz, range_m, config.environment.MODEL, atmosphere);
                link_status.atmospheric_loss_db = atm_loss_db;

                % --- 8. Rain Loss (Patch 13 Placeholder) ---
                rain_loss_db = 0;
                if strcmpi(config.rf.rain_loss_model, 'itu_rain')
                     try
                         % ** Placeholder Call - Replace with proper implementation **
                         rain_loss_db = calculate_itu_rain_loss_placeholder(freq_hz, range_m, state.alt_m_msl, rx_cfg.alt_m_msl, state.lat_deg);
                          link_status.failure_reason = append_reason(link_status.failure_reason, sprintf('RainLossPlhdr=%.1fdB', rain_loss_db));
                     catch ME_rain, warning('[RF %s->%s] Rain Loss calc fail: %s', tx_id, rx_id, ME_rain.message); end
                end
                link_status.rain_loss_db = rain_loss_db;

                % --- 9. Polarization Mismatch Loss (Patch 13) ---
                pol_loss_db = config.rf.polarization_mismatch_loss_db; % Direct from config (constant value)
                link_status.polarization_loss_db = pol_loss_db;

                % --- 10. Multipath Effects (Patch 16 Placeholder) ---
                multipath_loss_db = 0; delay_spread_s = 0;
                if config.rf.ENABLE_MULTIPATH_MODEL
                    try
                        % ** Placeholder Call - Replace with proper implementation **
                         [multipath_loss_db, delay_spread_s] = calculate_multipath_effects_placeholder(config.rf.multipath_model_type, range_m, el_body_deg); % Pass elevation angle?
                         link_status.failure_reason = append_reason(link_status.failure_reason, sprintf('MPLossPlhdr=%.1fdB', multipath_loss_db));
                    catch ME_mp, warning('[RF %s->%s] Multipath calc fail: %s', tx_id, rx_id, ME_mp.message); end
                end
                link_status.multipath_loss_db = multipath_loss_db;
                link_status.delay_spread_s = delay_spread_s;

                % --- 11. Other Losses (Including Fresnel - Patch 13 Placeholder) ---
                other_losses_db = 0;
                if config.rf.fresnel_zone_clearance_check
                     try
                         % ** Placeholder Call - Replace with proper implementation **
                         % Requires Tx/Rx positions and terrain model object
                         tx_pos_llh_fr = earth.ECEF_to_LLH(tx_pos_ecef(1), tx_pos_ecef(2), tx_pos_ecef(3)); % Get Tx LLH again
                         is_fresnel_clear = check_fresnel_clearance_placeholder(tx_pos_llh_fr, [rx_cfg.lat_deg, rx_cfg.lon_deg, rx_cfg.alt_m_msl], terrain);
                         if ~is_fresnel_clear
                             fresnel_loss_db = 3.0; % Example 3dB loss if not clear
                             other_losses_db = other_losses_db + fresnel_loss_db;
                             link_status.failure_reason = append_reason(link_status.failure_reason, sprintf('FresnelPlhdr=%.1fdB', fresnel_loss_db));
                         end
                     catch ME_fr, warning('[RF %s->%s] Fresnel check fail: %s', tx_id, rx_id, ME_fr.message); end
                end
                % Add any other miscellaneous losses here if needed
                link_status.other_losses_db = other_losses_db;

                % --- 12. Total Propagation Loss ---
                total_loss_db = fspl_db + atm_loss_db + rain_loss_db + pol_loss_db + multipath_loss_db + other_losses_db;
                link_status.total_loss_db = total_loss_db;

                % --- 13. Received Power ---
                % Pr(dBm) = Pt(dBm) + Gt(dBi) + Gr(dBi) - Ltotal(dB)
                received_power_dbm = (tx_power_dbw + 30) + total_antenna_gain_dbi - total_loss_db;
                link_status.received_power_dbm = received_power_dbm;

                % --- 14. Noise Power ---
                % Pn(dBm) = 10*log10(k*T*B) + 30 + NF(dB)
                % Get bandwidth for this receiver, use default if not specified
                if ~isfield(rx_cfg, 'bandwidth_hz') || isempty(rx_cfg.bandwidth_hz)
                     bandwidth_hz = config.rf.default_bandwidth_hz;
                     if i_tx == 1 && i_rx == 1 % Warn only once per step if default is used
                          warning('[RF MODEL] Receiver "%s" missing bandwidth_hz, using default %.1f MHz for noise calcs.', rx_id, bandwidth_hz/1e6);
                     end
                else
                     bandwidth_hz = rx_cfg.bandwidth_hz;
                end
                noise_temp_k    = config.rf.noise_temperature_k;
                noise_figure_db = config.rf.noise_figure_db;
                noise_power_thermal_dbm = 10*log10(K_BOLTZMANN * noise_temp_k * bandwidth_hz) + 30;
                noise_power_dbm = noise_power_thermal_dbm + noise_figure_db;
                link_status.noise_power_dbm   = noise_power_dbm;
                link_status.bandwidth_hz      = bandwidth_hz;
                link_status.noise_figure_db   = noise_figure_db;

                % --- 15. Signal-to-Noise Ratio (SNR) ---
                snr_db = received_power_dbm - noise_power_dbm;
                link_status.snr_db = snr_db;

                % --- 16. Link Status Check vs Sensitivity ---
                rx_sensitivity_dbm = rx_cfg.sensitivity_dbm;
                link_status.rx_sensitivity_dbm = rx_sensitivity_dbm;
                link_status.required_snr_db    = rx_sensitivity_dbm - noise_power_dbm; % Implied required SNR

                % Check if received power is below the required sensitivity
                if received_power_dbm < rx_sensitivity_dbm && strcmp(link_status.status, 'OK') % Only fail if not already failed
                    link_status.status = 'FAIL';
                    link_status.failure_reason = append_reason(link_status.failure_reason, sprintf('Pr < Sens (%.1f<%.1f)', received_power_dbm, rx_sensitivity_dbm));
                    link_status.primary_failure_step = 'Signal Level';
                end

                % --- 17. Doppler Shift ---
                doppler_hz = 0;
                if config.rf.ENABLE_DOPPLER_SHIFT
                    try
                        % Assume ground station velocity is zero in ECEF
                        rx_vel_ecef = [0; 0; 0];
                        relative_velocity_ecef = ac_vel_ecef - rx_vel_ecef; % Relative velocity vector [3x1] ECEF
                        % Project relative velocity onto LOS vector (unit vector ECEF)
                        relative_speed_los = dot(relative_velocity_ecef, los_unit_ecef); % scalar, +ve = moving away
                        % Calculate Doppler shift: fd = -vr * f / c
                        doppler_hz = -(relative_speed_los / C_LIGHT) * freq_hz; % +ve shift if closing range
                    catch ME_dop
                        warning('[RF %s->%s] Doppler calc fail: %s', tx_id, rx_id, ME_dop.message); doppler_hz=NaN;
                        link_status.failure_reason = append_reason(link_status.failure_reason, 'Doppler Error');
                    end
                end
                link_status.doppler_shift_hz = doppler_hz;

                % --- 18. Shannon Capacity (Theoretical Limit) ---
                capacity_bps = 0;
                % Calculate only if SNR is valid (even if link failed sensitivity check)
                if isfinite(snr_db) && bandwidth_hz > 0
                    snr_linear = 10^(snr_db / 10);
                    if snr_linear < 0, snr_linear = 0; end % Ensure non-negative linear SNR
                    capacity_bps = bandwidth_hz * log2(1 + snr_linear); % bps
                end
                link_status.shannon_capacity_bps = capacity_bps;

                % --- Final Failure Reason Consolidation ---
                if isempty(link_status.failure_reason) && strcmp(link_status.status, 'OK')
                    link_status.failure_reason = 'Link OK'; % Explicitly mark OK links
                    link_status.primary_failure_step = 'N/A';
                elseif isempty(link_status.failure_reason) && ~strcmp(link_status.status, 'OK') % Status is FAIL/PENDING but no reason set
                    link_status.failure_reason = 'Unknown Failure';
                    link_status.primary_failure_step = 'Unknown';
                elseif ~isempty(link_status.failure_reason) && strcmp(link_status.status, 'OK')
                    % If a reason was logged (e.g. placeholder warning) but status is still OK
                    link_status.failure_reason = ['Link OK (' link_status.failure_reason ')'];
                    link_status.primary_failure_step = 'N/A';
                end % Else: Status is FAIL and reason already set

            catch ME_link_calc % Catch errors during calculation for a specific link
                 warning('[RF MODEL %s->%s] Error during link calculation: %s. Marking link as FAIL.', tx_id, rx_id, ME_link_calc.message);
                 link_status.status = 'FAIL';
                 link_status.failure_reason = append_reason(link_status.failure_reason, 'Calculation Error');
                 link_status.primary_failure_step = 'Calculation Error';
            end % End try-catch for link calculation

            % Store final status struct for this link
            rf_status(link_idx) = link_status;

        end % End Rx loop
    end % End Tx loop

    % --- Aggregate Failure Summary ---
    try % Wrap summary calculation in try-catch
        valid_links_mask = ~strcmp({rf_status.status}, 'SKIP') & ~strcmp({rf_status.status}, 'PENDING');
        failed_links_mask = strcmp({rf_status(valid_links_mask).status}, 'FAIL');
        % Get primary failure steps only for links that were calculated AND failed
        failure_steps_list = {rf_status(valid_links_mask).primary_failure_step}; % Get steps for all valid links first
        failure_steps_list = failure_steps_list(failed_links_mask); % Filter for failed links only

        unique_steps = unique(failure_steps_list);
        failure_summary.counts = struct(); % Initialize counts struct
        failure_summary.total_calculated_links = sum(valid_links_mask);
        failure_summary.total_failed_links = sum(failed_links_mask);

        for k = 1:length(unique_steps)
            step_name = unique_steps{k};
            if isempty(step_name), continue; end % Skip if step name is empty
            count = sum(strcmp(failure_steps_list, step_name));
            field_name = matlab.lang.makeValidName(step_name); % Sanitize for field name
            failure_summary.counts.(field_name) = count;
        end
    catch ME_summary
         warning('Failed to generate failure summary: %s', ME_summary.message);
         failure_summary.error = ME_summary.message;
    end

end % END compute_rf_links_internal

% =========================================================================
% RF PLACEHOLDER & HELPER FUNCTIONS
% =========================================================================

% --- CREATE LINK STATUS STRUCT HELPER ---
function link_status = create_link_status_struct_helper()
    % Creates an empty structure with all fields pre-defined for a single link status.
    % Ensures consistency across all links, even if some fields aren't calculated.
    link_status = struct( ...
        'time_s', NaN, 'tx_id', '', 'rx_id', '', ...
        'status', 'PENDING', 'failure_reason', '', 'primary_failure_step', '', ...
        'frequency_hz', NaN, 'range_m', NaN, ...
        'azimuth_body_deg', NaN, 'elevation_body_deg', NaN, ...
        'terrain_blocked', false, 'aircraft_masked', false, ...
        'tx_power_dbm', NaN, 'tx_gain_dbi', NaN, 'rx_gain_dbi', NaN, ...
        'fspl_db', NaN, 'atmospheric_loss_db', NaN, 'rain_loss_db', NaN, ...
        'polarization_loss_db', NaN, 'multipath_loss_db', NaN, 'other_losses_db', NaN, ...
        'total_loss_db', NaN, 'received_power_dbm', NaN, ...
        'noise_figure_db', NaN, 'bandwidth_hz', NaN, 'noise_power_dbm', NaN, ...
        'rx_sensitivity_dbm', NaN, 'required_snr_db', NaN, 'snr_db', NaN, ...
        'doppler_shift_hz', NaN, 'shannon_capacity_bps', NaN, 'delay_spread_s', NaN ...
    );
end

% --- ATMOSPHERIC LOSS PLACEHOLDER ---
function loss_dB = calculate_atmospheric_loss_placeholder(f_Hz, distance_m, model_name, atmosphere_obj)
    % ** PLACEHOLDER - Replace with proper ITU gas model (e.g., P.676) **
    % Uses simple frequency/distance dependent model based on name only.
    freq_GHz = f_Hz / 1e9; dist_km = distance_m / 1000; coeff = 0.005; % Default coeff
    switch lower(model_name), case 'hot', coeff=0.006; case 'cold', coeff=0.004; case 'humid', coeff=0.007; case 'conservative', coeff=0.008; end
    loss_dB = max(0, coeff * (freq_GHz^1.5) * dist_km);
    % warning('Using PLACEHOLDER for Atmospheric Loss calculation.');
end

% --- RAIN LOSS PLACEHOLDER ---
function loss_db = calculate_itu_rain_loss_placeholder(varargin)
    % ** PLACEHOLDER - Replace with proper ITU rain model (e.g., P.838) **
    % Requires rain rate data (R0.01) for the location, path geometry etc.
    warning('Using PLACEHOLDER for ITU Rain Loss calculation.');
    loss_db = 0.1; % Return a minimal dummy loss
end

% --- MULTIPATH PLACEHOLDER ---
function [mp_loss_db, delay_s] = calculate_multipath_effects_placeholder(model_type, range_m, elevation_deg)
    % ** PLACEHOLDER - Replace with proper multipath model **
    warning('Using PLACEHOLDER for Multipath Effects calculation.');
    mp_loss_db = 0; delay_s = 0;
    if strcmpi(model_type, 'simple_ricean')
        mp_loss_db = 1.0 * exp(-abs(elevation_deg)/15); % Slightly adjusted example
        delay_s = 0.5e-9 * exp(-abs(elevation_deg)/30);
    elseif strcmpi(model_type, 'terrain_aware')
        mp_loss_db = 2.0 * exp(-abs(elevation_deg)/8); % Higher loss example
        delay_s = 5e-9 * exp(-abs(elevation_deg)/15);
    end
    mp_loss_db = max(0, min(mp_loss_db, 8.0)); delay_s = max(0, delay_s); % Cap values
end

% --- FRESNEL ZONE PLACEHOLDER ---
function is_clear = check_fresnel_clearance_placeholder(tx_llh, rx_llh, terrain_model)
    % ** PLACEHOLDER - Replace with proper Fresnel zone calculation **
    % Requires calculating path profile vs. terrain and comparing to Fresnel radius.
    warning('Using PLACEHOLDER for Fresnel Zone Clearance check.');
    is_clear = true; % Assume clear for placeholder
end

% --- ANGLE TO DCM HELPER ---
function R_body2ned = angle2dcm_local(yaw, pitch, roll, order)
    % Calculates Direction Cosine Matrix from Euler Angles (ZYX order).
    % Assumes inputs are in RADIANS.
    cy=cos(yaw); sy=sin(yaw); cp=cos(pitch); sp=sin(pitch); cr=cos(roll); sr=sin(roll);
    if strcmpi(order, 'ZYX')
        R_body2ned=[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr; sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr; -sp, cp*sr, cp*cr];
    else, error('Rotation order %s not implemented in angle2dcm_local.', order); end
end

% --- APPEND REASON HELPER ---
function reason_out = append_reason(reason_in, new_msg)
    % Safely appends a new message to the failure reason string.
    if isempty(reason_in) || strcmp(reason_in, 'Link OK')
        reason_out = new_msg; % Start new message
    else
        reason_out = [reason_in, '; ', new_msg]; % Append with separator
    end
end