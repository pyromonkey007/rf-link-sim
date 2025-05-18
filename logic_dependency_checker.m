% =========================================================================
% LOGIC DEPENDENCY CHECKER (FINAL - Corrected v3)
% Version: FINAL 1.3
%
% Description:
%   Validates the simulation configuration settings loaded from `config_settings.m`.
%   It checks for logical consistency between different feature toggles and ensures
%   that required parameters or files exist if a feature is enabled. This helps
%   prevent runtime errors or unexpected simulation behavior due to misconfiguration.
%   Called from `main_simulation.m` after configuration is loaded.
%
% Functionality:
%   - Checks dependencies for RF Module, Terrain, Aircraft Masking, Visualization,
%     Real-Time Input, and Output settings.
%   - Verifies file paths for DEM, STL, Wireframe, and Theme files if relevant features are enabled.
%   - Checks for potential conflicts or nonsensical settings (e.g., masking enabled but method 'none').
%   - Validates receiver band support configuration against defined transmitters.
%   - Prints confirmations [✓], warnings [! Warning], or errors [!] to the console.
%   - Collects critical error messages and throws a single error at the end if any
%     critical issues were found, halting the simulation start.
%
% Key Fixes in this Version:
%   - Removed invalid 'nonlocal'/'assignin' logic from helper function.
%   - Corrected faulty logic for validating STL/Wireframe masking settings.
%   - Corrected field name check for radar configuration ('lat_deg').
%   - Added robust error throwing at the end if critical errors are detected.
%   - Corrected warning/error message formatting (using fprintf directly).
%
% Usage:
%   logic_dependency_checker(config);
%
% Input:
%   config - The main configuration structure loaded from `config_settings.m`.
%
% Output:
%   - Console messages indicating check results.
%   - Throws an error using the `error()` function if critical misconfigurations are detected.
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-05] - Fix STL check, radar field name.
% =========================================================================
function logic_dependency_checker(config)

    fprintf('--- Running Configuration Logic Dependency Check ---\n');
    error_count = 0;        % Counter for critical errors found
    warning_count = 0;      % Counter for non-critical warnings found
    error_messages = {};    % Cell array to store critical error messages for final report

    % --- Check Helper: Check if File Exists ---
    % This nested function remains valid and useful
    function exists = check_file(filepath)
        % Checks if a file exists at the given path. Returns true/false.
        % Avoids printing directly; caller uses print_check based on result.
        % Checks if filepath input itself is valid before calling isfile.
        if isempty(filepath) || (~ischar(filepath) && ~isstring(filepath))
             exists = false; % Invalid filepath input is treated as file not existing
        else
             exists = isfile(filepath); % Check if it's a valid file path
        end
    end % End of check_file helper function

    % =====================================================================
    % Dependency Checks Start Here
    % =====================================================================

    % --- RF Module Dependencies ---
    fprintf('Checking RF Module dependencies...\n');
    % Get the master RF enable flag (use isfield for robustness against missing fields)
    rf_enabled = isfield(config.rf, 'ENABLE_RF_MODULE') && config.rf.ENABLE_RF_MODULE;
    fprintf('  [✓] RF Module Enabled = %s\n', mat2str(rf_enabled)); % Using simple fprintf

    if rf_enabled
        % Check if Tx/Rx lists are populated if RF module is enabled
        if isempty(config.rf.transmitters), warning_count=warning_count+1; fprintf('  [!] Warning %d: RF Enabled, but NO transmitters defined.\n',warning_count); end
        if isempty(config.rf.ground_stations), warning_count=warning_count+1; fprintf('  [!] Warning %d: RF Enabled, but NO ground stations defined.\n',warning_count); end

        % Check Receiver Band Support Configuration Consistency
        if ~isempty(config.rf.transmitters) && ~isempty(config.rf.ground_stations)
            % Get all defined Transmitter IDs
             all_tx_ids_chk = cellfun(@(x) x.id, config.rf.transmitters, 'UniformOutput', false);
             any_rx_valid = false; % Flag if at least one Rx can receive *something*

             % Loop through each Ground Station definition
             for i_gs_chk = 1:length(config.rf.ground_stations)
                 gs_chk = config.rf.ground_stations{i_gs_chk};
                 gs_id_chk = gs_chk.id; % Get station ID for messages

                 % Check if the 'supported_tx_ids' field exists and is valid (non-empty cell array of strings)
                 valid_support_field = isfield(gs_chk, 'supported_tx_ids') && iscellstr(gs_chk.supported_tx_ids) && ~isempty(gs_chk.supported_tx_ids);
                 if ~valid_support_field % If field missing or invalid
                     warning_count=warning_count+1; fprintf('  [!] Warning %d: Ground station "%s" missing/invalid `supported_tx_ids` (must be non-empty cellstr).\n', warning_count, gs_id_chk);
                 else % Field exists and is valid, check content
                      % Check if the Tx IDs listed for this Rx actually exist in the transmitter list
                      invalid_tx_listed = setdiff(gs_chk.supported_tx_ids, all_tx_ids_chk); % Find listed IDs not in master Tx list
                      if ~isempty(invalid_tx_listed) % If any listed IDs don't exist
                          warning_count=warning_count+1; fprintf('  [!] Warning %d: Ground station "%s" lists unsupported/undefined Tx IDs: %s\n', warning_count, gs_id_chk, strjoin(invalid_tx_listed, ', '));
                      end
                      % Check if this Rx supports at least one *defined* transmitter
                      if ~isempty(intersect(gs_chk.supported_tx_ids, all_tx_ids_chk))
                          any_rx_valid = true; % Mark that at least one Rx has valid support
                      end
                 end % End check if support field is valid
             end % End loop through ground stations

             % Warn if no receiver seems capable of receiving any defined transmitter signals
             if ~any_rx_valid
                 warning_count=warning_count+1; fprintf('  [!] Warning %d: No ground station appears configured to support any of the currently defined transmitter IDs.\n', warning_count);
             end
        end % End check Tx/Rx lists non-empty
    else % RF Module Disabled
        % --- Check if dependent RF features are incorrectly enabled ---
        if isfield(config.rf,'ENABLE_DOPPLER_SHIFT') && config.rf.ENABLE_DOPPLER_SHIFT, warning_count=warning_count+1; fprintf('  [!] Warning %d: Doppler enabled but RF Module disabled.\n', warning_count); end
        if isfield(config.rf,'ENABLE_MULTIPATH_MODEL') && config.rf.ENABLE_MULTIPATH_MODEL, warning_count=warning_count+1; fprintf('  [!] Warning %d: Multipath enabled but RF Module disabled.\n', warning_count); end
        if isfield(config.rf,'atmospheric_loss_model') && ~strcmpi(config.rf.atmospheric_loss_model,'none'), warning_count=warning_count+1; fprintf('  [!] Warning %d: Atmos loss (%s) active but RF Module disabled.\n', warning_count, config.rf.atmospheric_loss_model); end
        if isfield(config.rf,'rain_loss_model') && ~strcmpi(config.rf.rain_loss_model,'none'), warning_count=warning_count+1; fprintf('  [!] Warning %d: Rain loss (%s) active but RF Module disabled.\n', warning_count, config.rf.rain_loss_model); end
        if isfield(config.rf,'fresnel_zone_clearance_check') && config.rf.fresnel_zone_clearance_check, warning_count=warning_count+1; fprintf('  [!] Warning %d: Fresnel check enabled but RF Module disabled.\n', warning_count); end
        % Add checks for other RF-dependent features if added...
    end % End RF Enabled/Disabled checks

    % --- Terrain Dependencies ---
    fprintf('Checking Terrain dependencies...\n');
    terrain_enabled = isfield(config.terrain, 'ENABLE_TERRAIN_LOS') && config.terrain.ENABLE_TERRAIN_LOS;
    fprintf('  [✓] Terrain LOS Checks Enabled = %s\n', mat2str(terrain_enabled));
    if terrain_enabled
        dem_exists = check_file(config.terrain.DEM_FILE_PATH);
        if ~dem_exists % DEM file is critical if terrain LOS is enabled
            error_count = error_count + 1;
            msg = sprintf('Terrain LOS enabled, but DEM file NOT FOUND or path invalid: %s', config.terrain.DEM_FILE_PATH);
            fprintf('  [!] Error %d: %s\n', error_count, msg); error_messages{end+1} = msg;
        else % DEM file exists
            fprintf('    [✓] DEM file specified exists: "%s"\n', config.terrain.DEM_FILE_PATH);
            % Check Mapping Toolbox dependency only if DEM exists and GeoTIFF use is intended
            use_geotiff    = isfield(config.terrain,'USE_GEOTIFF') && config.terrain.USE_GEOTIFF;
            ignore_toolbox = isfield(config.advanced,'IGNORE_TOOLBOX_DEPENDENCIES') && config.advanced.IGNORE_TOOLBOX_DEPENDENCIES;
            mapping_exists = license('test', 'map_toolbox') && ~isempty(ver('map')); % Check license and installation
            if use_geotiff && ~mapping_exists && ~ignore_toolbox
                warning_count = warning_count + 1;
                fprintf('  [!] Warning %d: USE_GEOTIFF is true, but Mapping Toolbox not found/licensed. Terrain loading may fail or attempt CSV fallback.\n', warning_count);
            end
        end
    end

    % --- Aircraft Masking Dependencies ---
    fprintf('Checking Aircraft Masking dependencies...\n');
    masking_enabled = isfield(config.masking, 'ENABLE_AIRCRAFT_MASKING') && config.masking.ENABLE_AIRCRAFT_MASKING;
    fprintf('  [✓] Aircraft Masking Enabled = %s\n', mat2str(masking_enabled));
    if masking_enabled
        method = 'none'; % Default if METHOD field is missing
        if isfield(config.masking,'METHOD'), method = config.masking.METHOD; end

        % ** Corrected Logic Block for Masking Validation **
        % 1. Check if the specified method string itself is valid
        is_valid_method = ismember(lower(method), {'stl', 'wireframe', 'none'});
        if ~is_valid_method
            error_count = error_count + 1;
            msg = sprintf('Invalid masking method specified: "%s". Must be ''stl'', ''wireframe'', or ''none''.', method);
            fprintf('  [!] Error %d: %s\n', error_count, msg); error_messages{end+1} = msg;
        else
            % Method string is valid, now check file based on method
            fprintf('    [✓] Masking method specified: %s\n', method);
            if strcmpi(method, 'stl')
                stl_exists = check_file(config.masking.STL_FILE);
                % STL file existence is critical if method is STL
                if ~stl_exists
                    error_count=error_count+1; msg=sprintf('STL masking selected, but STL file NOT FOUND or path invalid: %s', config.masking.STL_FILE); fprintf('  [!] Error %d: %s\n', error_count, msg); error_messages{end+1}=msg;
                else
                    fprintf('      [✓] STL file exists: %s\n', config.masking.STL_FILE);
                end
            elseif strcmpi(method, 'wireframe')
                wf_exists = check_file(config.masking.WIREFRAME_FILE);
                % Wireframe file non-existence is maybe just a warning?
                if ~wf_exists
                    warning_count=warning_count+1; fprintf('  [!] Warning %d: Wireframe masking selected, but wireframe file NOT FOUND or path invalid: %s\n', warning_count, config.masking.WIREFRAME_FILE);
                else
                     fprintf('      [✓] Wireframe file exists: %s\n', config.masking.WIREFRAME_FILE);
                end
            elseif strcmpi(method, 'none')
                 % Only issue note if method is explicitly 'none' while master masking toggle is true
                 warning_count=warning_count+1; fprintf('  [!] Warning %d: Aircraft masking enabled, but method is ''none''. No masking applied.\n', warning_count);
            end
        end
        % End Corrected Logic Block
    end

    % --- Visualization Dependencies ---
    fprintf('Checking Visualization dependencies...\n');
    % Get status of various plotting flags using isfield for safety
    post_plots       = isfield(config.visualization,'ENABLE_POST_SIM_PLOTS') && config.visualization.ENABLE_POST_SIM_PLOTS;
    replay           = isfield(config.visualization,'ENABLE_TACTICAL_REPLAY') && config.visualization.ENABLE_TACTICAL_REPLAY;
    ray_trace        = isfield(config.visualization,'ENABLE_RAY_TRACE_VISUALIZER') && config.visualization.ENABLE_RAY_TRACE_VISUALIZER;
    realtime_gen     = isfield(config.visualization,'ENABLE_REALTIME_PLOTTING') && config.visualization.ENABLE_REALTIME_PLOTTING;
    realtime_lookdep = isfield(config.visualization,'ENABLE_REALTIME_LOOKDEP_PLOT') && config.visualization.ENABLE_REALTIME_LOOKDEP_PLOT;
    realtime_3dac    = isfield(config.visualization,'ENABLE_REALTIME_3D_AIRCRAFT') && config.visualization.ENABLE_REALTIME_3D_AIRCRAFT;
    realtime_3dpath  = isfield(config.visualization,'ENABLE_REALTIME_3D_FLIGHT_PATH') && config.visualization.ENABLE_REALTIME_3D_FLIGHT_PATH;
    snapshots        = isfield(config.output,'ENABLE_SNAPSHOT_EXPORT') && config.output.ENABLE_SNAPSHOT_EXPORT;
    dashboard        = isfield(config.output,'ENABLE_HTML_DASHBOARD') && config.output.ENABLE_HTML_DASHBOARD;

    % Check if snapshots enabled without any plots to snapshot
    any_plotting_enabled = post_plots || realtime_gen || ray_trace || replay; % Check if any relevant plot source is enabled
    if snapshots && ~any_plotting_enabled
        warning_count=warning_count+1; fprintf('  [!] Warning %d: Snapshot export enabled, but no relevant plots/replay enabled.\n', warning_count);
    end

    % Check if theme file exists if cyberpunk style selected
    if strcmpi(config.visualization.PLOT_STYLE, 'cyberpunk')
        theme_exists = check_file(config.visualization.CYBERPUNK_THEME_FILE);
        if ~theme_exists
            warning_count=warning_count+1; fprintf('  [!] Warning %d: Cyberpunk theme selected, but theme file not found/invalid: %s. Standard style will be used.\n', warning_count, config.visualization.CYBERPUNK_THEME_FILE);
        else
             fprintf('    [✓] Cyberpunk theme file exists: %s\n', config.visualization.CYBERPUNK_THEME_FILE);
        end
    end

    % Check real-time look/dep plot dependency on radar config
    if realtime_lookdep
        % ** Corrected v3 Field Name Check: 'lat_deg' and other fields **
        radar_defined = isfield(config, 'radar') && ...
                        isfield(config.radar, 'lat_deg') && ~isempty(config.radar.lat_deg) && ...
                        isfield(config.radar, 'lon_deg') && ~isempty(config.radar.lon_deg) && ...
                        isfield(config.radar, 'alt_m_msl'); % Check all required fields exist and lat is non-empty
        if ~radar_defined
            error_count=error_count+1; msg='RT Look/Dep plot enabled, but config.radar (lat_deg,lon_deg,alt_m_msl) undefined/incomplete.'; fprintf('  [!] Error %d: %s\n', error_count, msg); error_messages{end+1}=msg;
        else
             fprintf('    [✓] Radar site configuration found for Look/Dep plot.\n');
        end
    end

    % Check if general RT plotting enabled but no specific RT plots
    if realtime_gen && ~realtime_3dac && ~realtime_3dpath && ~realtime_lookdep
        fprintf('  [i] Note: General real-time plotting enabled, but no specific real-time plots (3D Aircraft, 3D Path, Look/Dep) are enabled.\n');
    end

    % Check dashboard asset dependency
    if dashboard
        if ~isfolder('dashboard')
            warning_count=warning_count+1; fprintf('  [!] Warning %d: HTML Dashboard enabled, but required ''dashboard'' source asset folder not found in project root.\n', warning_count);
        else
            fprintf('    [✓] Dashboard asset source folder found.\n');
        end
    end

    % --- Real-Time Input Dependencies ---
    fprintf('Checking Real-Time Input dependencies...\n');
    rt_input_enabled = isfield(config.simulation,'ENABLE_REALTIME_INPUT') && config.simulation.ENABLE_REALTIME_INPUT;
    if rt_input_enabled
        % Check timeout value
        if config.simulation.realtime_input_timeout_s <= 0
            warning_count=warning_count+1; fprintf('  [!] Warning %d: RT input timeout <= 0 (%.1fs). Autopilot fallback immediate.\n', warning_count, config.simulation.realtime_input_timeout_s);
        end
        % Warn if no figure is likely to be open for capturing input
        any_vis_figure_likely = post_plots || replay || realtime_gen || realtime_lookdep; % Check if any vis option likely to create a figure
        if ~any_vis_figure_likely
            warning_count=warning_count+1; fprintf('  [!] Warning %d: RT input enabled, but no visualization active. Keyboard capture may fail.\n', warning_count);
        end
    else
         fprintf('  [✓] Real-time input disabled.\n');
    end

    % --- Output Dependencies ---
    fprintf('Checking Output dependencies...\n');
    % Folder existence checked/created in main_simulation and config_settings

    % =====================================================================
    % --- Final Summary & Error Throw ---
    % =====================================================================
    fprintf('--- Logic Check Summary: %d Errors, %d Warnings ---\n', error_count, warning_count);
    if error_count > 0
        fprintf(2, 'CRITICAL CONFIGURATION ERRORS FOUND (%d):\n', error_count); % Print summary in red to command window
        for i = 1:length(error_messages)
            fprintf(2, '  - %s\n', error_messages{i}); % Print each stored critical error message
        end
        % Throw a MATLAB error to halt execution, including the error count
        error('Halting simulation due to %d critical configuration error(s) listed above. Review config_settings.m.', error_count);
    elseif warning_count > 0
        fprintf('Logic check completed with %d warning(s). Review settings marked [!] Warning.\n', warning_count);
    else
        fprintf('Logic check completed successfully. Configuration appears consistent.\n');
    end
    fprintf('----------------------------------------------------\n');

end % END OF FUNCTION logic_dependency_checker