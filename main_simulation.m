% =========================================================================
% MAIN SIMULATION DRIVER (FINAL - Corrected v2, Re-Commented)
% Version: FINAL 1.1
%
% Description:
%   This script serves as the master entry point and controller for the
%   entire high-fidelity RF link simulation process. It orchestrates the
%   loading of configurations, initialization of simulation models, execution
%   of the time-step simulation loop (via aircraft_dynamics_model), post-
%   processing of results (including RF calculations), data exporting, and
%   calls to post-simulation visualization and analysis tools.
%
% Workflow:
%   1. Setup: Clear environment (optional), add project paths.
%   2. Configuration: Load settings from 'config_settings.m'.
%   3. Validation: Run 'logic_dependency_checker.m' (optional, recommended).
%   4. Output Prep: Ensure output folders exist, log config settings.
%   5. Model Init: Initialize Earth, Atmosphere, Terrain, Antenna models.
%   6. Simulation Run: Call 'aircraft_dynamics_model.m' to get the flight path (TSPI).
%   7. RF Calculation (Post-Processing): Recalculate RF link performance for
%      each step of the generated trajectory using 'rf_propagation_model.m'.
%   8. Results Processing: Convert raw RF log cell array into structured data.
%   9. Data Export: Save TSPI, Raw Logs using utility functions.
%  10. Visualization & Analysis: Call post-simulation plotting, tactical replay,
%      ray tracing, and HTML dashboard generation functions.
%  11. Cleanup: Display final messages.
%
% Key Fixes in this Version:
%   - Corrected MEXCEP errors: Formatted warning/error calls within try-catch blocks.
%   - Added comment explaining 'clear' command.
%   - Ensured proper indentation/alignment.
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-05] - Fix MEXCEP errors.
% =========================================================================
%function main_simulation() % Wrap main code in a function for better scoping and path management

% --- 1. Setup Environment ---
% Clear workspace, command window, close figures for a clean run
% Note: 'clear' can sometimes hinder debugging if you want to inspect
%       variables from a previous run, but ensures no old data interferes. (Code Analyzer Check: CLEAR0ARGS)
clear;
clc;
close all; % Close existing figures
fprintf('=============================================================\n');
fprintf(' High-Fidelity Aircraft RF Link Simulation - Initializing...\n');
fprintf('=============================================================\n');

% --- Add Project Subfolders to MATLAB Path ---
% Ensures MATLAB can find functions defined in subdirectories (models, utils, etc.)
try
    project_root = fileparts(mfilename('fullpath')); % Get folder where this script is located
    if isempty(project_root), project_root = pwd; end % Fallback if run directly from command line
    addpath(genpath(project_root)); % Add the root and all subfolders recursively
    fprintf('Added project subfolders to MATLAB path (root: %s).\n', project_root);

    addpath(fullfile(project_root, 'visualization')); % Explicitly add visualization path
fprintf('Explicitly added path: %s\n', fullfile(project_root, 'visualization'));

catch ME_path
    % ** FIX for MEXCEP: Format warning message correctly **
    warning(ME_path.identifier, 'Could not automatically add project paths: %s\nEnsure all subfolders (models, utils, etc.) are on the MATLAB path.', ME_path.message);
end

% --- 2. Load Configuration & Initial Validation ---
fprintf('Loading configuration settings...\n');
try
    % Calls the config script, which returns the 'config' structure
    % The config_settings function now includes basic validation checks.
    config = config_settings();
    fprintf('Configuration loaded and basic validation passed.\n');
catch ME_config
    % ** FIX for MEXCEP: Format error message correctly **
    error(ME_config.identifier, 'Failed to load or validate configuration from config_settings.m: %s', ME_config.message);
end

% --- 3. Detailed Logic Dependency Check ---
% Runs checks for conflicts or missing dependencies between different config settings.
if config.advanced.PERFORM_LOGIC_CHECK
    fprintf('Performing detailed logic dependency check...\n');
    try
        logic_dependency_checker(config); % This function will throw an error if critical issues found
        fprintf('[✓] Logic dependencies check passed.\n');
    catch ME_logic
        % Catch errors specifically from the logic checker function
        % The checker itself should format the error message before throwing
        fprintf(2, 'Logic Dependency Check Error: %s\n', ME_logic.message); % Print error message in red
        % Display stack trace for debugging
        for i=1:length(ME_logic.stack)
            fprintf(2,'  at %s, line %d\n', ME_logic.stack(i).file, ME_logic.stack(i).line);
        end
        error('MainSim:ConfigError','Simulation halted due to critical configuration errors detected by logic checker.');
    end
else
    fprintf('Skipping detailed logic dependency check.\n');
end

% --- 4. Output Folder Management & Config Logging ---
% Ensure output directories exist (should be created by config_settings, but double-check)
fprintf('Ensuring output folder exists: %s\n', config.output.FOLDER);
subfolders_to_check = {config.output.FOLDER, ...
    fullfile(config.output.FOLDER, 'figures'), ...
    fullfile(config.output.FOLDER, 'logs'), ...
    fullfile(config.output.FOLDER, 'dashboard')};
for i = 1:length(subfolders_to_check)
    if ~isfolder(subfolders_to_check{i})
        fprintf('Creating missing output subfolder: %s\n', subfolders_to_check{i});
        try mkdir(subfolders_to_check{i}); catch ME_mkdir, warning('MainSim:MkdirWarn','Failed to create %s: %s', subfolders_to_check{i}, ME_mkdir.message); end
    end
end

% --- Log Configuration Used for This Run ---
config_log_mat_path = fullfile(config.output.FOLDER, 'logs', 'config_log.mat');
config_summary_txt_path = fullfile(config.output.FOLDER, 'logs', 'config_summary.txt');
try
    save(config_log_mat_path, 'config'); % Save full config struct to .mat file
    fprintf('Saved configuration snapshot to: %s\n', config_log_mat_path);
    % Write human-readable summary using helper function defined at end of this file
    write_config_summary_helper(config, config_summary_txt_path);
    fprintf('Saved configuration summary to: %s\n', config_summary_txt_path);
catch ME_save_cfg
    % ** FIX for MEXCEP error: Format warning correctly **
    warning(ME_save_cfg.identifier, 'Failed to save configuration log files: %s', ME_save_cfg.message);
end

% --- 5. Initialize Core Simulation Models ---
% Creates instances of the model objects using settings from the config structure.
fprintf('Initializing simulation models...\n');
try
    earth      = earth_model(config.earth.model);              % WGS-84 etc.
    atmosphere = atmospheric_model(config.environment.MODEL); % Standard, Hot, Cold etc.
    terrain    = terrain_model(config.terrain);              % Handles DEM loading/disabling

    % Load antenna patterns - result is a struct where fields are Tx IDs
    antennas   = antenna_model_loader(config.rf.transmitters, config.antenna); %
    % Initialize RF Propagation model object (contains compute_rf handle)
    rf_model_post = rf_propagation_model(); % Used for post-simulation calculation
    fprintf('[✓] Core models initialized.\n'); %
catch ME_models %
    % ** FIX for MEXCEP error: Format error correctly **
    error(ME_models.identifier, 'Failed to initialize core simulation models: %s', ME_models.message); %
end %

% --- 6. Run Main Simulation Loop (Aircraft Dynamics) ---
% This executes the core flight path generation.
fprintf('Starting main simulation loop (Aircraft Dynamics)...\n');
simulation_start_time = tic; % Start timer for dynamics phase
% Initialize output variables from the dynamics model
tspi_data = []; rf_log_cell_dummy = {}; failure_log_cell_dummy = {}; simulation_metadata = struct();
try
    % === EXECUTION ===
    % Calls the aircraft dynamics model function.
    % NOTE: The returned rf_log_cell and failure_log_cell from this step are
    %       currently placeholders, as RF calcs are done post-simulation.
    [tspi_data, rf_log_cell_dummy, failure_log_cell_dummy, simulation_metadata] = ...
        aircraft_dynamics_model(config, earth, atmosphere, terrain, antennas); % Pass initialized models

    simulation_duration = toc(simulation_start_time);
    fprintf('\n[✓] Aircraft Dynamics Simulation loop completed successfully in %.2f seconds.\n', simulation_duration);
    if isfield(simulation_metadata, 'completion_reason')
        fprintf('Simulation ended: %s\n', simulation_metadata.completion_reason);
    end

catch ME_simulation % Handle errors during the simulation run
    simulation_duration = toc(simulation_start_time);
    fprintf('\n==================== SIMULATION DYNAMICS ERROR ====================\n');
    fprintf('Simulation failed during dynamics phase after %.2f seconds.\n', simulation_duration);
    % ** FIX for MEXCEP error: Format error correctly **
    fprintf(2,'Error reported: %s (%s)\n', ME_simulation.message, ME_simulation.identifier); % Print error message in red
    % Display stack trace for easier debugging
    for i=1:length(ME_simulation.stack)
        fprintf(2,'  at %s, line %d\n', ME_simulation.stack(i).file, ME_simulation.stack(i).line);
    end
    fprintf('=====================================================================\n');

    % Attempt to save partial TSPI data if some was generated before the error
    if ~isempty(tspi_data) && config.output.ENABLE_TSPI_EXPORT
        fprintf('Attempting to save partial TSPI data...\n');
        try
            headers = {}; if isfield(simulation_metadata,'tspi_headers'), headers = simulation_metadata.tspi_headers; end
            if isempty(headers), warning('MainSim:PartialSave','Cannot save partial TSPI with headers, metadata missing.'); else
                export_TSPI_matrix(tspi_data, headers, config.output.FOLDER, config.output.TSPI_FORMAT, true); % Pass partial flag
                fprintf('Saved partial TSPI data.\n');
            end
        catch ME_save_partial
            % ** FIX for MEXCEP error: Format warning correctly **
            warning(ME_save_partial.identifier, 'Failed to save partial TSPI data: %s', ME_save_partial.message);
        end
    end
    % Rethrow error to stop script execution cleanly
    error('MainSim:RuntimeError','Simulation halted due to runtime error during dynamics phase.');
end

% --- 7. Post-Simulation RF Calculation ---
% Recalculate RF link performance for each step of the generated trajectory.
rf_log_cell = {}; failure_log_cell = {}; % Initialize actual log cell arrays
if isempty(tspi_data) || simulation_metadata.num_steps < 1
    fprintf('No valid TSPI data generated, skipping RF post-processing.\n');
    rf_data_available = false;
elseif config.rf.ENABLE_RF_MODULE
    fprintf('Calculating RF Propagation based on generated trajectory...\n');
    num_steps_actual = size(tspi_data, 1);
    rf_log_cell = cell(num_steps_actual, 1); % Preallocate actual log cells
    failure_log_cell = cell(num_steps_actual, 1);
    time_vector = tspi_data(:, 1); % Needed for RF log structure

    % Get TSPI column indices using headers for robustness
    hdrs = lower(simulation_metadata.tspi_headers);
    lat_col   = find(strcmp(hdrs,'lat_deg'),1); lon_col=find(strcmp(hdrs,'lon_deg'),1); alt_col=find(strcmp(hdrs,'alt_m_msl'),1);
    hdg_col   = find(strcmp(hdrs,'heading_deg'),1); pitch_col=find(strcmp(hdrs,'pitch_deg'),1); roll_col=find(strcmp(hdrs,'roll_deg'),1);
    gs_col    = find(strcmp(hdrs,'groundspeed_mps'),1); vs_col=find(strcmp(hdrs,'verticalspeed_mps'),1);
    % Check if essential columns were found
    if isempty(lat_col)||isempty(lon_col)||isempty(alt_col)||isempty(hdg_col)||isempty(pitch_col)||isempty(roll_col)||isempty(gs_col)||isempty(vs_col)
        warning('MainSim:TSPIHeadersMissing','Cannot perform post-sim RF calculation: Missing required columns (Lat/Lon/Alt/Att/Speed) in TSPI based on headers.');
        rf_data_available = false; % Mark RF data as unavailable
    else
        fprintf('Processing %d time steps...\n', num_steps_actual);
        update_interval = max(1, floor(num_steps_actual / 10)); % Update progress roughly every 10%
        for i_step = 1:num_steps_actual
            % Reconstruct the 'state' structure needed by rf_propagation_model for this time step
            current_state = struct();
            current_state.lat_deg       = tspi_data(i_step, lat_col);
            current_state.lon_deg       = tspi_data(i_step, lon_col);
            current_state.alt_m_msl     = tspi_data(i_step, alt_col);
            current_state.heading_deg   = tspi_data(i_step, hdg_col);
            current_state.pitch_deg     = tspi_data(i_step, pitch_col);
            current_state.roll_deg      = tspi_data(i_step, roll_col);
            % Reconstruct velocity vector [N; E; D] from Ground Speed, Vertical Speed, Heading
            gs = tspi_data(i_step, gs_col); vs = tspi_data(i_step, vs_col); hdg = current_state.heading_deg;
            gs_N = gs * cosd(hdg); gs_E = gs * sind(hdg); v_D = -vs; % Estimate Vd = -VS
            current_state.velocity_ned_mps = [gs_N; gs_E; v_D];

            % Call RF calculation function from the initialized rf_model object
            try
                [rf_log_cell{i_step}, failure_log_cell{i_step}] = rf_model_post.compute_rf(...
                    config, current_state, earth, atmosphere, terrain, antennas, time_vector(i_step));
            catch ME_rf_step
                warning(ME_rf_step.identifier,'Error during post-sim RF calc at step %d: %s. Logging empty.', i_step, ME_rf_step.message); % ** FIX for MEXCEP error **
                rf_log_cell{i_step} = struct('status','Calculation Error'); failure_log_cell{i_step} = struct('reason',ME_rf_step.message);
            end

            % Display progress periodically
            if mod(i_step, update_interval) == 0
                fprintf('  RF Calculation Progress: %.0f%%\n', (i_step/num_steps_actual)*100);
            end
        end % End loop through time steps
        fprintf('Post-simulation RF calculation complete.\n');
        rf_data_available = true; % Mark RF data as available
    end % End check for required TSPI columns
else
    fprintf('RF Module disabled, skipping post-simulation RF calculation.\n');
    rf_data_available = false;
end % End RF Module enabled check

% --- 8. Process Simulation Results (Structure RF Log) ---
fprintf('Processing simulation results...\n');
processed_rf_data = struct(); % Initialize empty struct

if rf_data_available && ~isempty(rf_log_cell) % Proceed only if RF was calculated
    % Process the RF Log Cell Array into a structured format for easier plotting/analysis
    % Uses helper function defined at end of this file
    processed_rf_data = post_process_rf_log_helper(config, rf_log_cell, time_vector);
    if isempty(fieldnames(processed_rf_data))
        fprintf('Processed RF data structure is empty. RF plots/stats may be missing.\n');
        rf_data_available = false; % Update flag if processing failed
    end
else
    fprintf('No valid RF data to process.\n');
    rf_data_available = false;
end % End RF log processing check

% =====================================================================
% --- 9. Export Final Data ---
% =====================================================================

% --- Export TSPI Data ---
if config.output.ENABLE_TSPI_EXPORT && ~isempty(tspi_data)
    fprintf('Exporting TSPI data...\n');
    try
        % Pass the headers directly from simulation_metadata
        if ~isfield(simulation_metadata, 'tspi_headers') || isempty(simulation_metadata.tspi_headers)
            warning('MainSim:TSPIExportWarn','TSPI headers missing from metadata. Cannot export TSPI with headers.');
        else
            export_TSPI_matrix(tspi_data, simulation_metadata.tspi_headers, config.output.FOLDER, config.output.TSPI_FORMAT);
            fprintf('[✓] TSPI data exported successfully.\n');
        end
    catch ME_export
        % ** FIX for MEXCEP error: Format warning correctly **
        warning(ME_export.identifier, 'Failed to export TSPI data: %s', ME_export.message);
    end
else
    fprintf('Skipping TSPI data export (disabled or no data).\n');
end

% --- Export Raw Data Logs ---
if config.output.ENABLE_FULL_RAW_DATA_LOG && ~isempty(tspi_data)
    fprintf('Exporting detailed raw data logs...\n');
    raw_log_path = fullfile(config.output.FOLDER, 'logs', ['raw_simulation_logs.' config.output.RAW_DATA_FORMAT]);
    try
        % Save all relevant data for offline analysis/debugging/re-plotting
        % Includes config, raw trajectory, raw cell logs, processed RF struct, metadata
        save(raw_log_path, 'config', 'tspi_data', 'rf_log_cell', 'failure_log_cell', 'processed_rf_data', 'simulation_metadata', '-v7.3');
        fprintf('[✓] Raw data logs saved to %s\n', raw_log_path);
    catch ME_save_raw
        % ** FIX for MEXCEP error: Format warning correctly **
        warning(ME_save_raw.identifier, 'Failed to save detailed raw data logs: %s', ME_save_raw.message);
    end
else
    fprintf('Skipping detailed raw data log export (disabled or no data).\n');
end

% =====================================================================
% --- 10. Post-Simulation Visualization & Analysis ---
% =====================================================================

% --- Generate Standard Summary Plots ---
if config.visualization.ENABLE_POST_SIM_PLOTS && ~isempty(tspi_data)
    fprintf('Generating post-simulation plots...\n');
    try
        fprintf('\n--- Checking path before visualization_model ---\n');
% Display the calculated project root used for genpath
try
    fprintf('Calculated project_root for genpath: %s\n', project_root);
catch
    fprintf('Could not display project_root variable (may not exist in this scope yet).\n');
end

% Call the main plotting function with processed data
        visualization_model(config, tspi_data, processed_rf_data, terrain, simulation_metadata);
        fprintf('[✓] Post-simulation plots generated.\n');
    catch ME_vis
        % ** FIX for MEXCEP error: Format warning correctly **
        warning(ME_vis.identifier, 'Failed to generate post-simulation plots: %s', ME_vis.message);
        % Display stack trace for easier debugging of plotting errors
        for i=1:length(ME_vis.stack), fprintf(2,'  Plotting Error at %s, line %d\n', ME_vis.stack(i).file, ME_vis.stack(i).line); end
    end
else
    fprintf('Skipping post-simulation plots (disabled or no data).\n');
end

% --- Launch Tactical Replay Mode ---
if config.visualization.ENABLE_TACTICAL_REPLAY && ~isempty(tspi_data)
    fprintf('Launching Tactical Replay Mode (Close replay window to continue)...\n');
    try
        tactical_replay_mode(config, tspi_data, processed_rf_data, terrain, simulation_metadata);
        fprintf('[✓] Tactical Replay Mode finished.\n');
    catch ME_replay
        % ** FIX for MEXCEP error: Format warning correctly **
        warning(ME_replay.identifier, 'Failed to launch/run Tactical Replay Mode: %s', ME_replay.message);
        for i=1:length(ME_replay.stack), fprintf(2,'  Replay Error at %s, line %d\n', ME_replay.stack(i).file, ME_replay.stack(i).line); end
    end
else
    fprintf('Skipping Tactical Replay Mode (disabled or no data).\n');
end

% --- Generate Static Ray Trace Visualization Snapshot ---
% Can be called here to generate a snapshot at a specific time index (e.g., end time)
if config.visualization.ENABLE_RAY_TRACE_VISUALIZER && ~isempty(tspi_data)
    fprintf('Generating final Ray Trace plot...\n');
    try
        ray_trace_visualizer(config, tspi_data, processed_rf_data, terrain, simulation_metadata); % Defaults to last time index
        fprintf('[✓] Final Ray Trace plot generated.\n');
    catch ME_ray
        % ** FIX for MEXCEP error: Format warning correctly **
        warning(ME_ray.identifier, 'Failed to generate final Ray Trace plot: %s', ME_ray.message);
    end
end

% --- Generate HTML Dashboard ---
if config.output.ENABLE_HTML_DASHBOARD && ~isempty(tspi_data) % Check data exists
    fprintf('Generating HTML Dashboard...\n');
    try
        % Ensure visualization path is added if generate_html_dashboard is there
        % addpath('visualization'); % Should be handled by genpath at start
        generate_html_dashboard(config, simulation_metadata, processed_rf_data); % Call the generator
        fprintf('[✓] HTML Dashboard generated successfully.\n');
    catch ME_dashboard
        % ** FIX for MEXCEP error: Format warning correctly **
        warning(ME_dashboard.identifier, 'Failed to generate HTML dashboard: %s', ME_dashboard.message);
        for i=1:length(ME_dashboard.stack), fprintf(2,'  Dashboard Error at %s, line %d\n', ME_dashboard.stack(i).file, ME_dashboard.stack(i).line); end
    end
else
    fprintf('Skipping HTML Dashboard generation (disabled or no data).\n');
end

% =====================================================================
% --- 11. Final Cleanup & Messages ---
% =====================================================================
fprintf('=============================================================\n');
fprintf(' Simulation Run Finished.\n');
fprintf(' Output data saved in: %s\n', config.output.FOLDER);
fprintf('=============================================================\n');

% Optional: Remove project paths if added at the start
% rmpath(genpath(project_root)); disp('Removed project paths.');

%end % END OF FUNCTION main_simulation


% =========================================================================
% HELPER FUNCTION: Write Config Summary Text File
% =========================================================================
function write_config_summary_helper(config, filepath)
% Writes a human-readable summary of key config parameters to a text file.
fid = -1; % Initialize file ID
try
    fid = fopen(filepath,'w'); % Open file for writing
    if fid == -1
        warning('MainSimHelper:FileOpenError','Could not open config summary file for writing: %s', filepath);
        return; % Cannot proceed if file won't open
    end

    fprintf(fid, 'Simulation Configuration Summary - %s\n\n', datetime('now','Format','yyyy-MM-dd HH:mm:ss')); % Use datetime
    fprintf(fid, 'Output Folder: %s\n\n', config.output.FOLDER);

    % --- Simulation Parameters ---
    fprintf(fid, '--- Simulation ---\n');
    if isfield(config.simulation,'duration_s'), fprintf(fid, 'Duration: %.1f s\n', config.simulation.duration_s); end
    if isfield(config.simulation,'time_step_s'), fprintf(fid, 'Time Step: %.3f s\n', config.simulation.time_step_s); end
    if isfield(config.simulation,'ENABLE_REALTIME_INPUT'), fprintf(fid, 'Realtime Input: %s\n', mat2str(config.simulation.ENABLE_REALTIME_INPUT)); end
    if isfield(config.simulation,'END_AT_LAST_WAYPOINT'), fprintf(fid, 'End at Last WP: %s\n\n', mat2str(config.simulation.END_AT_LAST_WAYPOINT)); end

    % --- Flight Parameters ---
    fprintf(fid, '--- Flight ---\n');
    if isfield(config.flight,'waypoints'), fprintf(fid, 'Num Waypoints: %d\n\n', size(config.flight.waypoints, 1)); end

    % --- Environment Parameters ---
    fprintf(fid, '--- Environment ---\n');
    if isfield(config.environment,'MODEL'), fprintf(fid, 'Atmosphere Model: %s\n', config.environment.MODEL); end
    fprintf(fid, 'Wind Enabled: %s (%.1f kts from %.0f deg)\n', ...
        mat2str(isfield(config.environment,'wind_speed_kts') && config.environment.wind_speed_kts > 0), ...
        config.environment.wind_speed_kts, config.environment.wind_direction_deg);
    if isfield(config.environment,'enable_gusts'), fprintf(fid, 'Gusts Enabled: %s\n\n', mat2str(config.environment.enable_gusts)); end

    % --- RF Parameters ---
    fprintf(fid, '--- RF --- (Enabled: %s)\n', mat2str(config.rf.ENABLE_RF_MODULE));
    if isfield(config.rf,'transmitters'), fprintf(fid, 'Num Transmitters: %d\n', length(config.rf.transmitters)); end
    if isfield(config.rf,'ground_stations'), fprintf(fid, 'Num Ground Stations: %d\n', length(config.rf.ground_stations)); end
    fprintf(fid, 'Doppler: %s, Multipath: %s (%s)\n', ...
        mat2str(config.rf.ENABLE_DOPPLER_SHIFT), ...
        mat2str(config.rf.ENABLE_MULTIPATH_MODEL), config.rf.multipath_model_type);
    fprintf(fid, 'Atmos Loss: %s, Rain Loss: %s\n\n', ...
        config.rf.atmospheric_loss_model, config.rf.rain_loss_model);

    % --- Terrain Parameters ---
    fprintf(fid, '--- Terrain --- (LOS Enabled: %s)\n', mat2str(config.terrain.ENABLE_TERRAIN_LOS));
    fprintf(fid, 'DEM File: %s\n\n', config.terrain.DEM_FILE_PATH);

    % --- Masking Parameters ---
    fprintf(fid, '--- Masking --- (Enabled: %s)\n', mat2str(config.masking.ENABLE_AIRCRAFT_MASKING));
    fprintf(fid, 'Method: %s\n\n', config.masking.METHOD);

    % --- Visualization Parameters ---
    fprintf(fid, '--- Visualization ---\n');
    fprintf(fid, 'Post Plots: %s, Replay: %s, Ray Trace: %s, Dashboard: %s\n', ...
        mat2str(config.visualization.ENABLE_POST_SIM_PLOTS), ...
        mat2str(config.visualization.ENABLE_TACTICAL_REPLAY), ...
        mat2str(config.visualization.ENABLE_RAY_TRACE_VISUALIZER), ...
        mat2str(config.output.ENABLE_HTML_DASHBOARD));
    fprintf(fid, 'Realtime Plots: %s (3D AC: %s, 3D Path: %s, Look/Dep: %s)\n', ...
        mat2str(config.visualization.ENABLE_REALTIME_PLOTTING), ...
        mat2str(config.visualization.ENABLE_REALTIME_3D_AIRCRAFT), ...
        mat2str(config.visualization.ENABLE_REALTIME_3D_FLIGHT_PATH), ...
        mat2str(config.visualization.ENABLE_REALTIME_LOOKDEP_PLOT));
    fprintf(fid, 'Plot Style: %s\n', config.visualization.PLOT_STYLE);

    % Close the file
    fclose(fid);

catch ME_write_sum % Catch errors during file writing
    % ** FIX for MEXCEP error: Format warning correctly **
    warning(ME_write_sum.identifier, 'Could not write config summary file "%s": %s', filepath, ME_write_sum.message);
    if fid ~= -1, try fclose(fid); catch; end; end % Ensure file is closed on error
end
end % END write_config_summary_helper

% =========================================================================
% HELPER: Post-Process RF Log Cell Array
% =========================================================================
function processed_rf_data = post_process_rf_log_helper(config, rf_log_cell, time_vector)
% Processes the raw rf_log_cell (output from post-simulation RF calculation)
% into a more easily usable structure, organized by TxID -> RxID -> Parameter.
% Handles skipped links gracefully.

processed_rf_data = struct(); % Initialize output structure

% Basic checks on input
if ~isfield(config.rf,'ENABLE_RF_MODULE') || ~config.rf.ENABLE_RF_MODULE || ...
        isempty(rf_log_cell) || ~iscell(rf_log_cell) || isempty(time_vector)
    fprintf('  Skipping RF log post-processing (RF disabled or logs empty).\n');
    return; % Nothing to process
end
num_steps = length(time_vector);

try % Wrap processing in try-catch
    % Find the first non-empty log entry to determine structure and link count
    first_log_entry = []; first_valid_step = 1;
    while isempty(first_log_entry) && first_valid_step <= length(rf_log_cell)
        first_log_entry = rf_log_cell{first_valid_step};
        first_valid_step = first_valid_step + 1;
    end

    % Check if a valid first entry was found
    if isempty(first_log_entry) || ~isstruct(first_log_entry)
        warning('MainSimHelper:RFLogInvalid', 'RF log cell array contains no valid structure entries. Cannot process RF data.');
        return;
    end

    num_links_total = length(first_log_entry); % Total number of potential links per time step
    field_names     = fields(first_log_entry); % Get parameter names from the first entry

    % --- V1.7 FIX: Remove 'time_s' from list of fields to process in loops ---
    time_s_idx = strcmp(field_names, 'time_s'); % Find index of 'time_s'
    if any(time_s_idx)
        fprintf('  INFO: Removing redundant ''time_s'' field from processing loop.\n');
        field_names(time_s_idx) = []; % Remove 'time_s' from the list
    end
    % --- END V1.7 FIX ---

    % --- Initialize the output structure ---
    % Preallocate fields based on the first log entry for ALL potential links
    for i_link = 1:num_links_total
        % ... (rest of initialization loop, lines 30-62, unchanged) ...
        % Get Tx and Rx IDs and sanitize them for use as struct field names
        tx_id = matlab.lang.makeValidName(first_log_entry(i_link).tx_id);
        rx_id = matlab.lang.makeValidName(first_log_entry(i_link).rx_id);
        % Create struct hierarchy if it doesn't exist
        if ~isfield(processed_rf_data, tx_id), processed_rf_data.(tx_id) = struct(); end
        if ~isfield(processed_rf_data.(tx_id), rx_id)
            processed_rf_data.(tx_id).(rx_id) = struct();
            processed_rf_data.(tx_id).(rx_id).time_s = time_vector; % Add time vector once per link
            % Preallocate arrays for each field based on data type
            for i_field = 1:length(field_names) % field_names now excludes time_s
                field = field_names{i_field};
                % Check if field actually exists in the first entry (robustness)
                if ~isfield(first_log_entry(i_link), field), continue; end
                value = first_log_entry(i_link).(field); % Get sample value
                % --- MODIFIED PREALLOCATION LOGIC ---
                force_cell_fields = {'status', 'failure_reason', 'primary_failure_step', 'tx_id', 'rx_id'}; % Add other fields if they might contain text
                if any(strcmp(field, force_cell_fields)) || ischar(value) || isstring(value) || iscell(value)
                    processed_rf_data.(tx_id).(rx_id).(field) = cell(num_steps, 1);
                    processed_rf_data.(tx_id).(rx_id).(field)(:) = {''}; % Use empty string default
                    if strcmp(field, 'status')
                         processed_rf_data.(tx_id).(rx_id).(field)(:) = {'PENDING'};
                    end
                elseif isnumeric(value) && isscalar(value)
                    processed_rf_data.(tx_id).(rx_id).(field) = NaN(num_steps, 1);
                elseif islogical(value) && isscalar(value)
                    processed_rf_data.(tx_id).(rx_id).(field) = false(num_steps, 1);
                end % End type check
                % --- END MODIFIED LOGIC ---
            end % End field loop for preallocation
        end % End check if receiver struct exists
    end % End initialization loop

    % --- Populate the Time Series Data ---
    % Loop through each time step in the raw cell log
    for i_step = 1:num_steps
        % ... (keep step_log assignment and validation, lines 66-76) ...
        step_log = rf_log_cell{i_step}; % Get the struct array for this step
        if isempty(step_log) || ~isstruct(step_log) || length(step_log) ~= num_links_total
            if i_step == 1, warning('MainSimHelper:RFLogStepInvalid','Invalid RF log entry at step %d.', i_step); end % Warn once
            continue;
        end
        % Loop through each link within the time step
        for i_link = 1:num_links_total
             % ... (keep tx_id, rx_id assignment, lines 79-80) ...
            tx_id = matlab.lang.makeValidName(step_log(i_link).tx_id);
            rx_id = matlab.lang.makeValidName(step_log(i_link).rx_id);
            % Check if this link was skipped (e.g., due to band filtering)
            is_skipped = strcmp(step_log(i_link).status, 'SKIP');

            % Loop through each parameter field for this link (field_names now excludes time_s)
             for i_field = 1:length(field_names) % field_names now excludes time_s
                  field = field_names{i_field};

                  % --- Check destination exists (preallocated correctly) ---
                  destination_var_exists = isfield(processed_rf_data.(tx_id).(rx_id), field);

                  % --- Get source value (check field exists in source log too) ---
                  if ~isfield(step_log(i_link), field), continue; end % Skip if source doesn't have this field
                  value = step_log(i_link).(field);

                  % --- Wrap assignment logic in try-catch (KEEP INNER CATCH COMMENTED FOR NOW) ---
                  if destination_var_exists
                      % try % START INNER TRY-CATCH (Keep Commented Out)
                          current_destination = processed_rf_data.(tx_id).(rx_id).(field);
                          if is_skipped
                              if any(strcmp(field, {'status','failure_reason','primary_failure_step','tx_id','rx_id'}))
                                  if iscell(current_destination)
                                      current_destination{i_step} = value;
                                  else
                                      current_destination{i_step} = value; % Attempt brace, let it error if prealloc failed
                                  end
                                  processed_rf_data.(tx_id).(rx_id).(field) = current_destination; % Assign back
                              end
                          else
                              if iscell(current_destination)
                                  current_destination{i_step} = value;
                                  processed_rf_data.(tx_id).(rx_id).(field) = current_destination; % Assign back
                              elseif (isnumeric(value) || islogical(value)) && isscalar(value)
                                   current_destination(i_step) = value;
                                   processed_rf_data.(tx_id).(rx_id).(field) = current_destination; % Assign back
                              end % End type check for assignment
                          end % End is_skipped check
                      % catch ME_assign % CATCH error during assignment (Keep Commented Out)
                          % fprintf(2, '>>>>> Error during assignment in post_process_rf_log_helper <<<<<\n');
                          % fprintf(2, '      Step: %d, Tx: %s, Rx: %s, Field: %s\n', i_step, tx_id, rx_id, field);
                          % fprintf(2, '      Destination variable class: %s\n', class(current_destination));
                          % fprintf(2, '      Value to assign class: %s\n', class(value));
                          % if isscalar(value), try fprintf(2, '      Value: %s\n', mat2str(value)); catch; end; end
                          % fprintf(2, '      Error Message: %s\n', ME_assign.message); % This line would fail if ME_assign undefined
                          % fprintf(2, '>>>>> Resuming processing... <<<<<\n');
                      % end % END INNER TRY-CATCH (Keep Commented Out)
                  end % End destination_var_exists check
             end % End field loop
        end % End link loop
    end % End step loop
    fprintf('RF log processed successfully into structured format.\n');

catch ME_process_rf % Catch errors during the processing
    % --- V1.7 FIX: Correct warning message to use the correct error object ---
    warning(ME_process_rf.identifier, 'Error processing RF log data: %s. Processed RF data may be incomplete or empty.', ME_process_rf.message); % Use ME_process_rf.message
    ME_process_rf.getReport % Display stack trace for debugging
    processed_rf_data = struct(); % Return empty structure on error
end
end % END post_process_rf_log_helper