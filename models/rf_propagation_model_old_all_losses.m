% =========================================================================
% RF PROPAGATION MODEL (High Fidelity)
% Version: 1.2 (Merged and Refactored)
%
% Description:
%   Calculates the detailed RF link budget and performance metrics for
%   aircraft transmitter and ground station receiver pairs.
%   Includes high-fidelity RF propagation effects.
%
% Core Functionality:
%   - Iterates through Tx-Rx pairs, checks band support.
%   - Geometry Calculation (Range, LOS vectors).
%   - Coordinate Transforms (ECEF, NED, Body).
%   - Antenna Gain Lookup.
%   - LOS Checks (Terrain, Aircraft Masking).
%   - Detailed Link Budget:
%       - FSPL
%       - Atmospheric Loss (Gas, Fog/Cloud)
%       - Rain Loss
%       - Fresnel Effects
%       - Pointing Error Loss
%       - XPD Loss
%       - Ground Bounce, Multipath, Diffraction
%   - Received Power, Noise Power, SNR.
%   - Link Margin, Adaptive Modulation.
%   - BER/SER Estimation.
%   - Doppler Shift/Spread.
%   - Shannon Capacity.
%   - Status & Failure Reason Logging.
%
% Inputs:
%   config     - Full configuration structure.
%   state      - Aircraft state.
%   earth      - Earth model object.
%   atmosphere - Atmospheric model object.
%   terrain    - Terrain model object.
%   antennas   - Antenna models structure.
%   t_sim      - Simulation time (seconds).
%
% Outputs:
%   rf_status       - Detailed results for each link.
%   failure_summary - Summary of failure reasons.
%
% Placeholders:
%   Implementation needed for advanced models.
%
% =========================================================================
function rf_model_obj = rf_propagation_model()
    % Constructor for the RF model.
    rf_model_obj.compute_rf = @compute_rf_links_internal;
    fprintf('[RF MODEL] RF Propagation Model Initialized (v1.2 - Merged).\n');
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

    % --- Basic Checks ---
    if ~isfield(config, 'rf') || ~isfield(config.rf, 'ENABLE_RF_MODULE') || ~config.rf.ENABLE_RF_MODULE
        rf_status=struct('status','RF Disabled');
        failure_summary=struct('reason','RF Disabled');
        return;
    end
    if ~isfield(state,'velocity_ned_mps') || isempty(state.velocity_ned_mps) || length(state.velocity_ned_mps) ~= 3
        error('RF Model requires state.velocity_ned_mps.');
    end
    if isempty(config.rf.transmitters) || isempty(config.rf.ground_stations)
        warning('[RF MODEL] No Tx/Rx defined.');
        rf_status=struct('status','No Tx/Rx'); failure_summary=struct('reason','No Tx/Rx'); return;
    end

    % --- Initialize Output Structures ---
    num_tx = length(config.rf.transmitters);
    num_rx = length(config.rf.ground_stations);
    num_links = num_tx * num_rx;
    rf_status = repmat(create_link_status_struct_helper(), num_links, 1);
    link_idx = 0;
    failure_summary = struct();

    % --- Calculate Aircraft ECEF State ---
    try
        [ac_pos_ecef(1), ac_pos_ecef(2), ac_pos_ecef(3)] = earth.LLH_to_ECEF(state.lat_deg, state.lon_deg, state.alt_m_msl);
        R_ned2ecef = earth.NED_to_ECEF_matrix(state.lat_deg, state.lon_deg);
        ac_vel_ecef = R_ned2ecef * state.velocity_ned_mps(:);
    catch ME_ecef
        error('Failed to calculate aircraft ECEF state: %s', ME_ecef.message);
    end

    % =====================================================================
    % === Loop Through Each Transmitter ===
    % =====================================================================
    for i_tx = 1:num_tx
        tx_cfg = config.rf.transmitters{i_tx};
        tx_id = tx_cfg.id;
        tx_field_name = matlab.lang.makeValidName(tx_id);

        % --- Check Antenna Model Exists ---
        if ~isfield(antennas, tx_field_name)
            warning('[RF MODEL] Antenna model for "%s" not found. Skipping Tx.', tx_id);
            for i_rx_skip = 1:num_rx
                link_idx = link_idx + 1;
                rx_cfg_skip = config.rf.ground_stations{i_rx_skip};
                rf_status(link_idx).time_s=t_sim; 
                rf_status(link_idx).tx_id=tx_id;
                rf_status(link_idx).rx_id=rx_cfg_skip.id; 
                rf_status(link_idx).status='SKIP';
                rf_status(link_idx).failure_reason='Antenna Model Missing';
                rf_status(link_idx).primary_failure_step='Setup';
            end
            continue;
        end
        ant_model = antennas.(tx_field_name);
        freq_hz = tx_cfg.freq_hz;
        tx_power_dbw = tx_cfg.power_dbw;
        tx_offset_body = tx_cfg.offset_body_m(:);
        tx_polarization = '';
        if isfield(tx_cfg, 'polarization')
            tx_polarization = upper(tx_cfg.polarization);
        end % Store Tx polarization

        % --- Calculate Tx Antenna ECEF Position ---
        try
            R_body2ned = angle2dcm_local(state.heading_deg*DEG2RAD, state.pitch_deg*DEG2RAD, state.roll_deg*DEG2RAD, 'ZYX');
            tx_offset_ecef = R_ned2ecef * R_body2ned * tx_offset_body;
            tx_pos_ecef = ac_pos_ecef(:) + tx_offset_ecef;
        catch ME_tx_pos
            warning('[RF MODEL Tx=%s] Failed to calculate Tx position: %s. Skipping Tx.', tx_id, ME_tx_pos.message);
            remaining_rx = num_rx - (link_idx - (i_tx-1)*num_rx);
            for i_rx_skip=1:remaining_rx
                link_idx = link_idx + 1;
                rx_cfg_skip = config.rf.ground_stations{i_rx_skip + (num_rx-remaining_rx)};
                rf_status(link_idx).time_s=t_sim; rf_status(link_idx).tx_id=tx_id;
                rf_status(link_idx).rx_id=rx_cfg_skip.id; rf_status(link_idx).status='SKIP';
                rf_status(link_idx).failure_reason='Tx Position Error';
                rf_status(link_idx).primary_failure_step='Setup';
            end
            continue;
        end

        % =====================================================================
        % === Loop Through Each Ground Station (Receiver) ===
        % =====================================================================
        for i_rx = 1:num_rx
            link_idx = link_idx + 1;
            rx_cfg = config.rf.ground_stations{i_rx};
            rx_id = rx_cfg.id;
            rx_polarization = '';
            if isfield(rx_cfg, 'polarization')
                rx_polarization = upper(rx_cfg.polarization);
            end % Store Rx polarization

            % --- Initialize Status Struct for This Link ---
            link_status = rf_status(link_idx); % Get handle
            link_status.time_s = t_sim;
            link_status.tx_id = tx_id;
            link_status.rx_id = rx_id;
            link_status.frequency_hz = freq_hz;
            link_status.tx_power_dbm = tx_power_dbw + 30;
            link_status.status = 'PENDING';
            link_status.failure_reason = '';
            link_status.primary_failure_step = '';

            % --- Band Filtering Check ---
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
                rf_status(link_idx)=link_status;
                continue;
            end

            % --- If supported, proceed... ---
            link_status.status = 'OK'; % Assume OK

            try % Wrap link calculation

                % --- 1. Calculate Geometry ---
                [rx_pos_ecef(1), rx_pos_ecef(2), rx_pos_ecef(3)] = earth.LLH_to_ECEF(rx_cfg.lat_deg, rx_cfg.lon_deg, rx_cfg.alt_m_msl);
                los_vector_ecef = rx_pos_ecef(:) - tx_pos_ecef(:);
                range_m = norm(los_vector_ecef);
                link_status.range_m = range_m;
                if range_m < 1.0
                    link_status.status='FAIL';
                    link_status.failure_reason='Range < 1m';
                    link_status.primary_failure_step='Geometry';
                    rf_status(link_idx)=link_status;
                    continue;
                end
                los_unit_ecef = los_vector_ecef / range_m;

                % --- 2. Coordinate Transformations ---
                R_ecef2ned_ac = earth.ECEF_to_NED_matrix(state.lat_deg, state.lon_deg);
                los_vector_ned = R_ecef2ned_ac * los_vector_ecef;
                R_ned2body = angle2dcm_local(state.heading_deg*DEG2RAD, state.pitch_deg*DEG2RAD, state.roll_deg*DEG2RAD, 'ZYX')'; % Use helper
                los_vector_body = R_ned2body * los_vector_ned;

                az_body_nominal = mod(atan2d(los_vector_body(2), los_vector_body(1)), 360);
                el_body_nominal = atan2d(los_vector_body(3), sqrt(los_vector_body(1)^2 + los_vector_body(2)^2));

                link_status.azimuth_body_deg = az_body_nominal;   % Store nominal angles
                link_status.elevation_body_deg = el_body_nominal;

                % --- 3. Pointing Error (Tier 1) ---
                az_lookup = az_body_nominal;    % Start with nominal angles
                el_lookup = el_body_nominal;
                pointing_error_loss_db = 0;      % Initialize
                if isfield(config.rf, 'ENABLE_POINTING_ERROR') && config.rf.ENABLE_POINTING_ERROR
                    try
                        % ** IMPLEMENT Pointing Error Calculation HERE **
                        %
                        az_error = randn() * config.rf.pointing_error_std_dev_az_deg;
                        el_error = randn() * config.rf.pointing_error_std_dev_el_deg;
                        %
                        az_lookup = az_body_nominal + az_error;
                        el_lookup = el_body_nominal + el_error;
                        % 3. Calculate gain at nominal angles (for loss calculation)
                        gain_nominal = ant_model.get_gain(az_body_nominal, el_body_nominal);
                        % 4. Calculate gain at errored angles (used for link budget)
                        gain_actual = ant_model.get_gain(az_lookup, el_lookup);
                        % 5. Calculate pointing loss
                        pointing_error_loss_db = gain_nominal - gain_actual; % Loss is positive dB
                        % Ensure loss is non-negative
                        pointing_error_loss_db = max(0, pointing_error_loss_db);
                    catch ME_point
                        warning('[RF %s->%s] Pointing Error calc fail: %s', tx_id, rx_id, ME_point.message);
                        pointing_error_loss_db = 0; % Default to zero loss
                        az_lookup = az_body_nominal; el_lookup = el_body_nominal;
                    end
                end
                link_status.pointing_error_loss_db = pointing_error_loss_db;
                % Note: az_lookup, el_lookup now contain potentially errored angles for gain lookup.

                % --- 4. Antenna Gain Lookup ---
                tx_gain_dbi = ant_model.get_gain(az_lookup, el_lookup); % Use the potentially errored lookup angles
                rx_gain_dbi = rx_cfg.rx_gain_dbi;
                total_antenna_gain_dbi = tx_gain_dbi + rx_gain_dbi;
                link_status.tx_gain_dbi = tx_gain_dbi;
                link_status.rx_gain_dbi = rx_gain_dbi;

                % --- 5. Terrain LOS Check ---
                is_terrain_blocked = false;
                if isfield(config.terrain, 'ENABLE_TERRAIN_LOS') && config.terrain.ENABLE_TERRAIN_LOS && terrain.data_loaded
                    try
                        [tx_pos_llh(1), tx_pos_llh(2), tx_pos_llh(3)] = earth.ECEF_to_LLH(tx_pos_ecef(1), tx_pos_ecef(2), tx_pos_ecef(3));
                        is_terrain_blocked = ~terrain.check_LOS(tx_pos_llh(1), tx_pos_llh(2), tx_pos_llh(3), rx_cfg.lat_deg, rx_cfg.lon_deg, rx_cfg.alt_m_msl);
                    catch ME_terrain_los
                        warning('[RF %s->%s] Terrain LOS check fail: %s', tx_id, rx_id, ME_terrain_los.message);
                        is_terrain_blocked = true;
                        link_status.failure_reason = append_reason(link_status.failure_reason, 'TerrCheckErr');
                    end
                end
                link_status.terrain_blocked = is_terrain_blocked;
                if is_terrain_blocked
                    link_status.status = 'FAIL';
                    link_status.failure_reason = append_reason(link_status.failure_reason, 'TerrainBlk');
                    link_status.primary_failure_step = 'LOS - Terrain';
                end

                % --- 6. Aircraft Masking Check ---
                is_aircraft_masked = false;
                if isfield(config.masking, 'ENABLE_AIRCRAFT_MASKING') && config.masking.ENABLE_AIRCRAFT_MASKING && ~is_terrain_blocked % Only check if path not already blocked
                    masking_method = config.masking.METHOD;
                    try
                        tx_pos_mask = tx_offset_body;
                        %
                        rx_pos_relative_ecef = rx_pos_ecef(:) - ac_pos_ecef(:);
                        %
                        rx_pos_mask = R_ned2body * R_ecef2ned_ac * rx_pos_relative_ecef;
                        %

                        if strcmpi(masking_method, 'stl') %%% STL Masking
                            stl_filepath = config.masking.STL_FILE;
                            if isfile(stl_filepath)
                                is_aircraft_masked = stl_LOS_intersection(tx_pos_mask, rx_pos_mask, stl_filepath);
                            else
                                warning('[RF %s->%s] Masking STL missing: %s', tx_id, rx_id, stl_filepath);
                            end
                        elseif strcmpi(masking_method, 'wireframe') %%% Wireframe Masking
                            if isfield(config.masking,'WIREFRAME_POINTS') && ~isempty(config.masking.WIREFRAME_POINTS)
                                wf_points = config.masking.WIREFRAME_POINTS;
                                is_aircraft_masked = wireframe_OML_masking(tx_pos_mask, rx_pos_mask, wf_points, config.masking);
                            elseif isfile(config.masking.WIREFRAME_FILE)
                                try
                                    wf_points = readmatrix(config.masking.WIREFRAME_FILE);
                                    is_aircraft_masked = wireframe_OML_masking(tx_pos_mask, rx_pos_mask, wf_points, config.masking);
                                catch ME_wf
                                    warning('[RF %s->%s] WF read fail %s: %s', tx_id, rx_id, config.masking.WIREFRAME_FILE, ME_wf.message);
                                end
                            else
                                warning('[RF %s->%s] Masking WF missing.', tx_id, rx_id);
                            end
                        end
                    catch ME_masking
                        warning('[RF %s->%s] Masking check fail (%s): %s', tx_id, rx_id, masking_method, ME_masking.message);
                        is_aircraft_masked = false;
                        link_status.failure_reason = append_reason(link_status.failure_reason, 'MaskCheckErr');
                    end
                end
                link_status.aircraft_masked = is_aircraft_masked;
                if is_aircraft_masked && strcmp(link_status.status, 'OK')
                    link_status.status = 'FAIL';
                    link_status.failure_reason = append_reason(link_status.failure_reason, sprintf('AC Mask (%s)', config.masking.METHOD));
                    link_status.primary_failure_step = 'LOS - Aircraft';
                end

                % --- Retrieve Atmospheric Conditions ---
                atmos_cond = atmosphere.get_conditions(state.alt_m_msl);
                temperature_k = atmos_cond.temperature_k;
                pressure_pa = atmos_cond.pressure_pa;
                density_kg_m3 = atmos_cond.density_kg_m3;
            
                % --- Initialize Losses ---
                fspl_db = 0;
                gas_loss_db = 0;
                rain_loss_db = 0;
                fog_cloud_loss_db = 0;
                fresnel_obstr_loss_db = 0;
                xpd_loss_db = 0;
                ground_bounce_loss_db = 0;
                iono_loss_db = 0;
                tropo_loss_db = 0;
                diffraction_loss_db = 0;

                % --- 7. Free Space Path Loss (FSPL) ---
                fspl_db = 20*log10(range_m) + 20*log10(freq_hz) - 147.55;  % dB
                link_status.fspl_db = fspl_db;

                % --- 8. Atmospheric Gas Loss (Tier 1) ---
                if isfield(config.rf, 'ENABLE_GAS_LOSS') && config.rf.ENABLE_GAS_LOSS
                    try
                        % Use ITU-R P.676-12 for oxygen/water vapor absorption
                        freq_GHz = freq_hz / 1e9;
                        if isfield(config.environment, 'pressure_hPa')
                            pressure_hPa = config.environment.pressure_hPa;
                        else
                            pressure_hPa = 1013.25;  % Standard atmospheric pressure
                        end
                        if isfield(config.environment, 'temperature_K')
                            temperature_K = config.environment.temperature_K;
                        else
                            temperature_K = 290;      % Standard temperature (17 degC)
                        end
                        if isfield(config.environment, 'humidity_pct')
                            humidity_pct = config.environment.humidity_pct;
                        else
                            humidity_pct = 50;         % Example value
                        end
                        if isfield(link_status, 'elevation_body_deg')
                            elevation_angle_deg = link_status.elevation_body_deg;
                        else
                            elevation_angle_deg = 45;  % Example value
                        end

                        %
                        es = 6.112 * exp((17.67 * (temperature_K - 273.15)) / (temperature_K - 29.65));
                        e = humidity_pct / 100 * es;
                        rho_w = 216.7 * (e / temperature_K);  % Approx. absolute humidity in g/m^3

                        %
                        gamma_o = 0.1820 * pressure_hPa * freq_GHz^2 / ((freq_GHz^2 + 1.0^2));  % Oxygen approx
                        %
                        gamma_w = 0.003 * rho_w * freq_GHz^2 / ((freq_GHz^2 + 20^2));          % Water vapor approx
                        %
                        total_specific_attenuation = gamma_o + gamma_w;  % [dB/km]

                        %
                        if elevation_angle_deg > 0
                            if isfield(state, 'alt_m_msl')
                                h_tx = state.alt_m_msl;
                            else
                                h_tx = 10000;  % Default if undefined
                            end
                            path_km = range_m / 1000 / sind(elevation_angle_deg);
                        else
                            path_km = 60;      % Default worst-case
                        end

                        gas_loss_db = total_specific_attenuation * path_km;
                        link_status.gas_loss_db = gas_loss_db;
                    catch ME_gas
                        warning('[RF %s->%s] Gas Loss calc fail: %s', tx_id, rx_id, ME_gas.message);
                    end
                end
                if ~isfield(link_status, 'gas_loss_db')
                    link_status.gas_loss_db = 0;
                end

                % --- 9. Rain Loss (Tier 1) ---
                if isfield(config.rf, 'ENABLE_RAIN_LOSS') && config.rf.ENABLE_RAIN_LOSS
                    try
                        % ITU-R P.838-3 Rain Attenuation Model
                        if isfield(config.environment, 'rain_rate_mm_per_hr')
                            rain_rate = config.environment.rain_rate_mm_per_hr;
                        else
                            rain_rate = 1.0;  % Light rain as default
                        end
                        freq_GHz = freq_hz / 1e9;
                        el_deg = max(link_status.elevation_body_deg, 0.1);

                        % Get k and alpha parameters (ITU-R P.838-3 empirical fits)
                        if freq_GHz < 10
                            k = 0.0001 * freq_GHz^2.42;
                            alpha = 1.0;
                        elseif freq_GHz < 100
                            k = 0.0000387 * freq_GHz^2.81;
                            alpha = 0.912;
                        else
                            k = 0.00005 * freq_GHz^2.6;
                            alpha = 0.88;
                        end

                        % Horizontal attenuation (dB/km)
                        gamma_r = k * (rain_rate)^alpha;
                        %
                        path_km = range_m / 1000 / sind(el_deg);
                        r_eff = 1.0 / (1.0 + 0.78 * sqrt(path_km * gamma_r / freq_GHz) - 0.38 * (1 - exp(-2 * path_km)));
                        Leff_km = path_km * r_eff;

                        rain_loss_db = gamma_r * Leff_km;
                        link_status.rain_loss_db = rain_loss_db;
                    catch ME_rain
                        warning('[RF %s->%s] Rain Loss calc fail: %s', tx_id, rx_id, ME_rain.message);
                    end
                end
                if ~isfield(link_status, 'rain_loss_db')
                    link_status.rain_loss_db = 0;
                end

                % --- 10. Fog / Cloud Loss (Tier 2) ---
                if isfield(config.rf, 'ENABLE_FOG_CLOUD_LOSS') && config.rf.ENABLE_FOG_CLOUD_LOSS
                    try
                        % Placeholder: A more detailed model would consider cloud density,
                        % droplet size distribution, and frequency dependence.
                        fog_cloud_loss_db = 0;
                        % Implement a simple fog/cloud loss model using approximate Mie scattering.
                        % As an example, assume fog/cloud introduces 0.02 dB/km at the link frequency
                        % (purely illustrative).
                        freq_GHz = freq_hz / 1e9;
                        fog_specific_loss_db_per_km = 0.02 * freq_GHz; 
                        path_km = range_m / 1000;
                        fog_cloud_loss_db = fog_specific_loss_db_per_km * path_km;
                        link_status.fog_cloud_loss_db = fog_cloud_loss_db;
                    catch ME_fog
                        warning('[RF %s->%s] Fog/Cloud Loss calc fail: %s', tx_id, rx_id, ME_fog.message);
                    end
                end
                if ~isfield(link_status, 'fog_cloud_loss_db')
                    link_status.fog_cloud_loss_db = 0;
                end

                % --- 11. Fresnel Zone Effects (Tier 1) ---
                if isfield(config.rf, 'ENABLE_FRESNEL_LOSS') && config.rf.ENABLE_FRESNEL_LOSS
                    try
                        % Implement a simple Fresnel clearance check (1st Fresnel zone).
                       lambda_m = C_LIGHT / freq_hz;
                      d1 = range_m * 0.5;
                        d2 = range_m - d1;
                        fresnel_radius = sqrt((lambda_m*d1*d2)/(d1+d2));
                        % We'll do a simplified approach: assume partial obstruction fraction
                        % from terrain is small. Suppose we have a clearance factor (0 => blocked, 1 => clear).
                        % We'll approximate an additional loss if clearance < 0.6 * radius.
                        clearance_ratio = 1.0; 
                        if terrain.data_loaded
                            % A real approach would check terrain alt. For demonstration:
                            clearance_ratio = 0.95; % e.g. partial clearance
                        end
                        if clearance_ratio < 0.6
                            fresnel_obstr_loss_db = 15; % Arbitrary penalty for poor clearance
                        else
                            fresnel_obstr_loss_db = 0;
                        end
                        link_status.fresnel_obstruction_loss_db = fresnel_obstr_loss_db;
                    catch ME_fresnel
                        warning('[RF %s->%s] Fresnel calc fail: %s', tx_id, rx_id, ME_fresnel.message);
                    end
                end
                if ~isfield(link_status, 'fresnel_obstruction_loss_db')
                    link_status.fresnel_obstruction_loss_db = 0;
                end

                % --- 12. Pointing Error Loss (Tier 1) ---
                if isfield(config.rf, 'ENABLE_POINTING_ERROR') && config.rf.ENABLE_POINTING_ERROR
                    % (Already calculated in step 3)
                end

                % --- 13. Cross-Polarization Discrimination (XPD) Loss (Tier 3) ---
                if isfield(config.rf, 'ENABLE_XPD_LOSS') && config.rf.ENABLE_XPD_LOSS
                    try
                        % Placeholder: A more detailed model would consider antenna orientations and polarization mismatch.
                        xpd_loss_db = 0;
                        % Implement a basic cross-polar mismatch model:
                        if ~isempty(tx_polarization) && ~isempty(rx_polarization) && ~strcmp(tx_polarization, rx_polarization)
                            % For orthogonal linear polarizations, assume ~20 dB mismatch
                            xpd_loss_db = 20;
                        else
                            xpd_loss_db = 0;
                        end
                        link_status.xpd_loss_db = xpd_loss_db;
                    catch ME_xpd
                        warning('[RF %s->%s] XPD Loss calc fail: %s', tx_id, rx_id, ME_xpd.message);
                    end
                end
                if ~isfield(link_status, 'xpd_loss_db')
                    link_status.xpd_loss_db = 0;
                end

                % --- 14. Ground Bounce Loss (Tier 2) ---
                if isfield(config.rf, 'ENABLE_GROUND_BOUNCE') && config.rf.ENABLE_GROUND_BOUNCE
                    try
                        % Implement a simple 2-ray ground reflection model:
                        lambda_m = C_LIGHT / freq_hz;
                        % Effective reflection coefficient (assuming average ground reflection)
                        R_ground = -0.6; 
                        phase_diff = 2*pi*(2*range_m)/lambda_m; 
                        ground_bounce_loss_db = -20*log10(abs(1 + R_ground*exp(-1i*phase_diff)));
                        link_status.ground_bounce_loss_db = max(0, ground_bounce_loss_db); 
                    catch ME_gb
                        warning('[RF %s->%s] Ground Bounce calc fail: %s', tx_id, rx_id, ME_gb.message);
                    end
                end
                if ~isfield(link_status, 'ground_bounce_loss_db')
                    link_status.ground_bounce_loss_db = 0;
                end

                % --- 15. Ionospheric Loss (Tier 3) ---
                if isfield(config.rf, 'ENABLE_IONOSPHERIC_EFFECTS') && config.rf.ENABLE_IONOSPHERIC_EFFECTS
                    try
                        % Placeholder: Requires TEC data and a model for ionospheric absorption and scintillation.
                        iono_loss_db = 0;
                        % Very rough approximation of ionospheric absorption (low at GHz)
                        % E.g., assume some small constant for L-band and above.
                        iono_loss_db = 0.1; 
                         link_status.iono_loss_db = iono_loss_db;
                    catch ME_iono
                        warning('[RF %s->%s] Ionospheric Loss calc fail: %s', tx_id, rx_id, ME_iono.message);
                    end
                end
                if ~isfield(link_status, 'iono_loss_db')
                    link_status.iono_loss_db = 0;
                end

                % --- 16. Troposcatter / Ducting Loss (Tier 3) ---
                if isfield(config.rf, 'ENABLE_TROPO_DUCTING') && config.rf.ENABLE_TROPO_DUCTING
                    try
                        % Placeholder: Requires a model (e.g., ITU-R P.452) and atmospheric conditions.
                        tropo_loss_db = 0;
                        % Implement a simple troposcatter model (ITU-R P.452 style):
                        % For demonstration, assume a small constant for modest range:
                        if range_m > 50000
                            tropo_loss_db = 10; 
                        else
                            tropo_loss_db = 0;
                        end
                        link_status.tropo_loss_db = tropo_loss_db;
                    catch ME_tropo
                        warning('[RF %s->%s] Tropo/Ducting Loss calc fail: %s', tx_id, rx_id, ME_tropo.message);
                    end
                end
                 if ~isfield(link_status, 'tropo_loss_db')
                    link_status.tropo_loss_db = 0;
                end

                % --- 17. Terrain Diffraction Loss (Tier 3) ---
                if isfield(config.rf, 'ENABLE_TERRAIN_DIFFRACTION') && config.rf.ENABLE_TERRAIN_DIFFRACTION
                    try
                        % Placeholder: Requires a path profile and a diffraction model (e.g., ITU-R P.526).
                        diffraction_loss_db = 0;
                         % Implement a simple diffraction model (e.g., ITU-R P.526 knife-edge):
                        % We'll assume if there's partial terrain blocking, add a typical diffraction penalty:
                        if is_terrain_blocked
                            diffraction_loss_db = 20; 
                        else
                            diffraction_loss_db = 0;
                        end
                        link_status.diffraction_loss_db = diffraction_loss_db;
                    catch ME_diff
                        warning('[RF %s->%s] Diffraction Loss calc fail: %s', tx_id, rx_id, ME_diff.message);
                    end
                 end
                 if ~isfield(link_status, 'diffraction_loss_db')
                    link_status.diffraction_loss_db = 0;
                end

                % --- Sum All Losses ---
                total_loss_db = fspl_db + gas_loss_db + rain_loss_db + fog_cloud_loss_db + ...
                              fresnel_obstr_loss_db + pointing_error_loss_db + xpd_loss_db + ...
                              ground_bounce_loss_db + iono_loss_db + tropo_loss_db + diffraction_loss_db;
                link_status.total_loss_db = total_loss_db;

                % --- 18. Received Power (Recalculate with all losses) ---
                received_power_dbm = (tx_power_dbw + 30) + total_antenna_gain_dbi - total_loss_db;
                link_status.received_power_dbm = received_power_dbm;

                % --- 19. Noise Power ---
                if ~isfield(rx_cfg, 'bandwidth_hz') || isempty(rx_cfg.bandwidth_hz)
                    bandwidth_hz = config.rf.default_bandwidth_hz;
                    if i_tx==1 && i_rx==1
                        warning('[RF] Rx "%s" missing BW, using default %.1f MHz.', rx_id, bandwidth_hz/1e6);
                    end
                else
                    bandwidth_hz = rx_cfg.bandwidth_hz;
                end
                noise_temp_k = config.rf.noise_temperature_k;
                noise_figure_db = config.rf.noise_figure_db;
                noise_power_thermal_dbm = 10*log10(K_BOLTZMANN * noise_temp_k * bandwidth_hz) + 30;
                noise_power_dbm = noise_power_thermal_dbm + noise_figure_db;
                link_status.noise_power_dbm = noise_power_dbm;
                link_status.bandwidth_hz = bandwidth_hz;
                link_status.noise_figure_db = noise_figure_db;

                % --- 20. SNR ---
                snr_db = received_power_dbm - noise_power_dbm;
                link_status.snr_db = snr_db;

                % --- 21. Statistical Fading (Multipath Fading - Tier 2) ---
                snr_db_eff = snr_db;  % Effective SNR used for BER/Modulation, starting with average SNR
                fading_loss_db = 0;
                if isfield(config.rf, 'ENABLE_MULTIPATH_FADING') && config.rf.ENABLE_MULTIPATH_FADING
                    try
                         % Instead of applying a fixed fade margin, implement a Rayleigh or Ricean model.
                        % 1) Check which model is desired from config.
                        if isfield(config.rf, 'MULTIPATH_FADING_MODEL')
                            fading_model = lower(config.rf.MULTIPATH_FADING_MODEL); % 'rayleigh' or 'ricean'
                        else
                            fading_model = 'rayleigh'; % Default
                        end
                        
                        % 2) For geometry, we can use the LOS presence to decide Rayleigh vs. Ricean automatically:
                        %    e.g. if line-of-sight is definitely present => Ricean, else => Rayleigh
                        %    But let's assume user sets it in config for clarity.
                        
                        switch fading_model
                            case 'rayleigh'
                                % Rayleigh fading => SNR penalty is random, typically negative-exponential distribution in amplitude
                                % We'll sample a single fade for this time step.
                                % Example approach: average fade of X dB with a random penalty on top.
                                % This is a simplified approach. More advanced would do time-series or distribution sampling.
                                
                                % Let's assume an average fade margin of ~5 dB and random variance ±2 dB
                                average_fade_db = 5;
                                random_variation_db = 2 * randn(); 
                                total_fade_db = average_fade_db + random_variation_db; 
                                
                                % Ensure the fade is not less than 0 dB for this approach
                                total_fade_db = max(0, total_fade_db);
                                
                                snr_db_eff = snr_db - total_fade_db;
                                fading_loss_db = total_fade_db;
                                
                            case 'ricean'
                                % Ricean fading => there's a dominant LOS component plus a scattered component
                                % The Rician K-factor determines the ratio between direct path power and scatter power.
                                
                                if isfield(config.rf, 'RICEAN_K_FACTOR')
                                    k_factor = config.rf.RICEAN_K_FACTOR; 
                                else
                                    k_factor = 6; % dB (example)
                                end
                                
                                % Convert K-factor dB => linear
                                K_linear = 10^(k_factor/10);
                                % A common approach is to model an RMS fade about an LOS average. 
                                % We'll do a small random fluctuation around that. 
                                % For demonstration, let's do a random fluctuation of ±2 dB around a base fade of 2 dB.
                                
                                base_fade_db = 2; 
                                random_variation_db = 2 * randn(); 
                                total_fade_db = base_fade_db + random_variation_db - (k_factor*0.1); 
                                % The higher the K-factor, the smaller the fade penalty
                                
                                total_fade_db = max(0, total_fade_db); 
                                snr_db_eff = snr_db - total_fade_db;
                                fading_loss_db = total_fade_db;
                                
                            otherwise
                                % Fallback to a simple margin
                                fade_margin_db = 3;
                                snr_db_eff = snr_db - fade_margin_db;
                                fading_loss_db = fade_margin_db;
                        end
                    catch ME_fade
                        warning('[RF %s->%s] Multipath Fading calc fail: %s', tx_id, rx_id, ME_fade.message);
                        snr_db_eff = snr_db;  % Revert to average SNR
                        fading_loss_db = 0;
                    end
                end
                link_status.fading_loss_db = fading_loss_db;
                % Use snr_db_eff for subsequent checks

                % --- 22. Link Status Check vs Sensitivity ---
                rx_sensitivity_dbm = rx_cfg.sensitivity_dbm;
                link_status.rx_sensitivity_dbm = rx_sensitivity_dbm;
                required_snr_db = rx_sensitivity_dbm - noise_power_dbm;  % Implied required SNR for basic link
                link_status.required_snr_db = required_snr_db;
                % Check effective SNR against implied requirement (Sensitivity - NoiseFloor)
                % Use snr_db_eff here as it includes fading effects.
                if snr_db_eff < required_snr_db && strcmp(link_status.status, 'OK')
                    link_status.status = 'FAIL';
                    link_status.failure_reason = append_reason(link_status.failure_reason, sprintf('EffSNR < Req (%.1f<%.1f)', snr_db_eff, required_snr_db));
                    link_status.primary_failure_step = 'Signal Level';
                end

                % --- 23. Adaptive Modulation & Data Rate (Tier 1) ---
                link_status.modulation_state = 'N/A';  % Default if not enabled/calculated
                link_status.data_rate_bps = 0;        % Default if not enabled/calculated
                if isfield(config.rf, 'ENABLE_ADAPTIVE_MODULATION') && config.rf.ENABLE_ADAPTIVE_MODULATION && isfield(config, 'modulation') && ~isempty(config.modulation.schemes)
                    try
                        % ** IMPLEMENT Adaptive Modulation Selection HERE **
                        % 1. Iterate through config.modulation.schemes (highest rate first).
                        % 2. Select the highest rate scheme where snr_db_eff >= scheme.min_snr_db.
                        % 3. Store the selected scheme's 'name' in link_status.modulation_state.
                        % 4. Store the selected scheme's 'data_rate_mbps' * 1e6 in link_status.data_rate_bps.
                        %
                        selected_scheme = 'N/A';
                        data_rate = 0;
                        for i_mod = 1:length(config.modulation.schemes)
                            scheme = config.modulation.schemes{i_mod};
                            if snr_db_eff >= scheme.min_snr_db
                                selected_scheme = scheme.name;
                                data_rate = scheme.data_rate_mbps * 1e6;
                                break;  % Found highest valid scheme
                            end
                        end
                        link_status.modulation_state = selected_scheme;
                        link_status.data_rate_bps = data_rate;

                        if strcmp(selected_scheme, 'N/A') && strcmp(link_status.status, 'OK')
                            link_status.status = 'FAIL';
                            link_status.failure_reason=append_reason(link_status.failure_reason, 'No Valid Mod');
                            link_status.primary_failure_step='Modulation';
                        end
                    catch ME_mod
                        warning('[RF %s->%s] Adaptive Modulation fail: %s', tx_id, rx_id, ME_mod.message);
                    end
                end

                % --- 24. BER / SER Estimation (Tier 1) ---
                link_status.ber = NaN;
                link_status.ser = NaN;  % Defaults
                if isfield(config.rf, 'ENABLE_BER_SER_ESTIMATION') && config.rf.ENABLE_BER_SER_ESTIMATION && isfield(config, 'modulation') && ~isempty(config.modulation.schemes) && ~strcmp(link_status.modulation_state, 'N/A')
                    try
                        % ** IMPLEMENT BER/SER Estimation HERE **
                        % 1. Find the 'ber_snr_curve' for the link_status.modulation_state.
                        %    - Use log interpolation for BER axis?  interp1(snr_vals, log10(ber_vals), snr_db_eff).
                        % 4. (Optional) Estimate SER if symbol size is known for the modulation.
                        % A small log interpolation example for BER:
                        snr_points = [0 5 10 15 20 25 30];
                        ber_points = [1e-1 1e-2 1e-3 1e-4 1e-5 1e-6 1e-7];
                        if snr_db_eff < snr_points(1)
                            link_status.ber = ber_points(1);
                        elseif snr_db_eff > snr_points(end)
                            link_status.ber = ber_points(end);
                        else
                            link_status.ber = 10^(interp1(snr_points, log10(ber_points), snr_db_eff));
                        end
                        link_status.ser = link_status.ber * 2; 
                    catch ME_ber
                        warning('[RF %s->%s] BER Estimation fail: %s', tx_id, rx_id, ME_ber.message);
                    end
                end

                % --- 25. Link Margin (Tier 1) ---
                link_status.link_margin_db = NaN;  % Default
                if isfield(config.rf, 'ENABLE_LINK_MARGIN_CALC') && config.rf.ENABLE_LINK_MARGIN_CALC
                    %
                    link_status.link_margin_db = received_power_dbm - rx_sensitivity_dbm;
                    % current_mod_min_snr = get_min_snr_for_modulation(link_status.modulation_state, config.modulation.schemes);
                    % link_status.modulation_margin_db = snr_db_eff - current_mod_min_snr;
                end

                % --- 26. Doppler Shift ---
                doppler_hz = 0;
                if isfield(config.rf, 'ENABLE_DOPPLER_SHIFT') && config.rf.ENABLE_DOPPLER_SHIFT % Re-check flag? Already done? No harm.
                    try
                        rx_vel_ecef = [0; 0; 0];
                        relative_velocity_ecef = ac_vel_ecef - rx_vel_ecef;
                        los_unit_ecef = los_vector_ecef / range_m;
                        relative_speed_los = dot(relative_velocity_ecef, los_unit_ecef);
                        doppler_hz = -(relative_speed_los / C_LIGHT) * freq_hz;
                    catch ME_dop
                        warning('[RF %s->%s] Doppler calc fail: %s', tx_id, rx_id, ME_dop.message);
                    end
                end
                link_status.doppler_shift_hz = doppler_hz;

                % --- 27. Doppler Spread (Tier 3) ---
                link_status.doppler_spread_hz = NaN;  % Default
                if isfield(config.rf, 'ENABLE_DOPPLER_SPREAD_CALC') && config.rf.ENABLE_DOPPLER_SPREAD_CALC
                    try
                         % Use a more advanced approach to Doppler spread
                        % Often, Doppler spread = (v_rel / c) * freq_center * angle_spread
                        % or a distribution-based approach (Clark model, Jakes model, etc.) 
                        
                        rx_vel_ecef = [0; 0; 0]; % If ground station is fixed
                        relative_velocity_ecef = ac_vel_ecef - rx_vel_ecef;
                        
                        % Magnitude of relative velocity
                        rel_speed_mps = norm(relative_velocity_ecef);
                        
                        % For an omni scattering environment, the maximum Doppler shift = rel_speed_mps/c * freq_hz.
                        f_max = (rel_speed_mps / C_LIGHT) * freq_hz;
                        
                        % The actual spread depends on the angular spread. Let's assume 
                        % a 360-degree environment => about 2*f_max total spread in worst case (Jakes model).
                        doppler_spread_hz = 2 * f_max; 
                        
                        % If you want a narrower spread (less multipath angle), you can scale it:
                        if isfield(config.rf, 'DOPPLER_ANGLE_SPREAD_DEG')
                            angle_spread_deg = config.rf.DOPPLER_ANGLE_SPREAD_DEG;
                            % E.g., scale proportionally:
                            doppler_spread_hz = f_max * (angle_spread_deg / 180); 
                        end
                        
                        link_status.doppler_spread_hz = doppler_spread_hz;
                    catch ME_dopspread
                        warning('[RF %s->%s] Doppler Spread calc fail: %s', tx_id, rx_id, ME_dopspread.message);
                    end
                end

                % --- 28. Shannon Capacity ---
                capacity_bps = 0;
                if isfield(config.rf, 'ENABLE_SHANNON_CAPACITY') && config.rf.ENABLE_SHANNON_CAPACITY && isfinite(snr_db) && bandwidth_hz > 0  % Use average SNR for theoretical capacity
                    snr_linear = 10^(snr_db / 10);
                    if snr_linear < 0, snr_linear = 0; end
                    capacity_bps = bandwidth_hz * log2(1 + snr_linear);
                end
                link_status.shannon_capacity_bps = capacity_bps;

                % --- 29. Outage, Margin Logging, and Summary ---
                if isfield(config.rf, 'ENABLE_SUMMARY_LOGGING') && config.rf.ENABLE_SUMMARY_LOGGING
                    if isfield(link_status, 'link_margin_db') && ~isnan(link_status.link_margin_db) && link_status.link_margin_db < 0
                        link_status.failure_reason = 'Fade Margin Exceeded';
                        link_status.status = 'FAIL';
                    end

                    % Optional outage burst tagging
                    link_status.outage_flag = strcmp(link_status.status, 'FAIL');
                    %link_status.total_loss_db = sum(loss_components_db);

                    % Attach label-wise breakdown for post analysis
                    %link_status.loss_labels = loss_labels;
                    %link_status.loss_values_db = loss_components_db;
                end

                % --- 30. Final Cleanup and NaN Handling ---
                fields = fieldnames(link_status);
                for f = 1:length(fields)
                    val = link_status.(fields{f});
                    if isnumeric(val) && any(isnan(val), 'all')
                        link_status.(fields{f}) = 0;  % Replace NaNs with 0
                    end
                end

                % --- 31. Final Loss Summary and Output Struct Logging ---
                %link_status.total_loss_db = sum(loss_components_db);
                %link_status.loss_breakdown = struct();
                %for i_loss = 1:length(loss_labels)
                %    sanitized_label = matlab.lang.makeValidName(loss_labels{i_loss});
                %    link_status.loss_breakdown.(sanitized_label) = loss_components_db(i_loss);
                %end

            catch ME_link_calc  % Catch errors during calculation for a specific link
                warning('[RF MODEL %s->%s] Error during link calculation: %s. Marking link FAIL.', tx_id, rx_id, ME_link_calc.message);
                link_status.status = 'FAIL';
                link_status.failure_reason = append_reason(link_status.failure_reason, 'CalcError');
                link_status.primary_failure_step = 'Calculation Error';
                link_status.tx_id = tx_id;
                link_status.rx_id = rx_id;
                link_status.time_s = t_sim;
            end % End try-catch for link calculation

            % Store final status struct for this link
            rf_status(link_idx) = link_status;
        end % End Rx loop
    end % End Tx loop

    % --- Aggregate Failure Summary ---
    try
        valid_links_mask = ~strcmp({rf_status.status}, 'SKIP') & ~strcmp({rf_status.status}, 'PENDING');
        failed_links_mask = strcmp({rf_status(valid_links_mask).status}, 'FAIL');
        failure_steps_list = {rf_status(valid_links_mask).primary_failure_step};
        failure_steps_list = failure_steps_list(failed_links_mask);

        unique_steps = unique(failure_steps_list);
        failure_summary.counts = struct();
        failure_summary.total_calculated_links = sum(valid_links_mask);
        failure_summary.total_failed_links = sum(failed_links_mask);
        for k = 1:length(unique_steps)
            step_name = unique_steps{k};
            if isempty(step_name), continue; end
            count = sum(strcmp(failure_steps_list, step_name));
            field_name = matlab.lang.makeValidName(step_name);
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

% --- CREATE LINK STATUS STRUCT HELPER (UPDATED for Fidelity) ---
function link_status = create_link_status_struct_helper()
    % Creates an empty structure with all fields pre-defined.
    link_status = struct( ...
        'time_s', NaN, 'tx_id', '', 'rx_id', '', ... % Identifiers
        'status', 'PENDING', 'failure_reason', '', 'primary_failure_step', '', ... % Status
        'frequency_hz', NaN, 'range_m', NaN, ... % Basic Geometry
        'azimuth_body_deg', NaN, 'elevation_body_deg', NaN, ... % Angles
        'terrain_blocked', false, 'aircraft_masked', false, ... % LOS Checks
        'tx_power_dbm', NaN, 'tx_gain_dbi', NaN, 'rx_gain_dbi', NaN, ... % Gains/Power
        'fspl_db', NaN, ... % Basic Loss
        ... % --- Fidelity Losses (dB) ---
        'gas_loss_db', 0, ...                  % Tier 1 - ITU P.676
        'rain_loss_db', 0, ...                 % Tier 1 - ITU P.838
        'fresnel_obstruction_loss_db', 0, ...  % Tier 1 - Calculated if obstructed
        'pointing_error_loss_db', 0, ...       %
        'xpd_loss_db', 0, ...                  % Tier 3 - Cross-polarization
        'fog_cloud_loss_db', 0, ...            % Tier 2 - ITU P.840
        'ground_bounce_loss_db', 0, ...        % Tier 2 - Effect of 2-Ray model
        'fading_loss_db', 0, ...               %
        'iono_loss_db', 0, ...                 % Tier 3 - Ionospheric effects
        'tropo_loss_db', 0, ...                % Tier 3 - Tropo/Ducting beyond LOS
        'diffraction_loss_db', 0, ...          % Tier 3 - Terrain diffraction
        'total_loss_db', NaN, ...              % Sum of all relevant losses
        ... % --- Link Budget Results ---
        'received_power_dbm', NaN, ...         % Pr
        'noise_figure_db', NaN, 'bandwidth_hz', NaN, 'noise_power_dbm', NaN, ... % Noise
        'rx_sensitivity_dbm', NaN, 'required_snr_db', NaN, ... % Thresholds
        'snr_db', NaN, ...                     % Average SNR (before fading)
        'link_margin_db', NaN, ...             %
        ... % --- Performance Metrics ---
        'modulation_state', 'N/A', ...         % Tier 1 - Selected modulation (string)
        'data_rate_bps', 0, ...                % Tier 1 - Achieved data rate (bps)
        'ber', NaN, ...                        % Tier 1 - Bit Error Rate estimate
        'ser', NaN, ...                        % Tier 1 - Symbol Error Rate estimate (optional)
        'shannon_capacity_bps', NaN, ...       % Tier 1 - Theoretical limit
        'doppler_shift_hz', NaN, ...           % Base Doppler
        'doppler_spread_hz', NaN, ...          % Tier 3 - Doppler spread estimate
        ... % --- Other ---
        'fresnel_clearance_factor', NaN ...    %
    );
    % Note: Removed 'delay_spread_s' as it wasn't explicitly requested like Doppler Spread
end

% --- ANGLE TO DCM HELPER ---
function R_body2ned = angle2dcm_local(yaw, pitch, roll, order)
    % Calculates Direction Cosine Matrix from Euler Angles (ZYX order).
    cy=cos(yaw); sy=sin(yaw); cp=cos(pitch); sp=sin(pitch); cr=cos(roll); sr=sin(roll);
    if strcmpi(order, 'ZYX')
        R_body2ned=[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr; sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr; -sp, cp*sr, cp*cr];
    else
        error('Rotation order %s not implemented.', order);
    end
end

% --- APPEND REASON HELPER ---
function reason_out = append_reason(reason_in, new_msg)
    % Safely appends a new message to the failure reason string.
    if isempty(reason_in) || strcmp(reason_in, 'Link OK')
        reason_out = new_msg;
    else
        reason_out = [reason_in, '; ', new_msg];
    end
end