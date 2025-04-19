% =========================================================================
% POST-SIMULATION VISUALIZATION MODEL (FINAL - Corrected v1.3)
% Version: FINAL 1.3
%
% Description:
%   Generates a suite of summary plots based on the completed simulation data.
%   This function is called by `main_simulation.m` after the main simulation
%   loop finishes and the RF log data has been processed into the
%   `processed_rf_data` structure. It visualizes key flight dynamics
%   parameters and RF link performance metrics using either standard MATLAB
%   plotting styles or a configured 'cyberpunk' theme. Handles cases where
%   RF data might be missing or specific links were skipped (e.g., due to
%   receiver band filtering). Calls `snapshot_generator` to save plots if enabled.
%
% Plots Generated:
%   (Standard Flight Dynamics)
%   1.  3D Flight Trajectory (with Waypoints, Ground Stations, Heading Arrows)
%   2.  Altitude vs. Time
%   3.  Speed (TAS/Ground Speed) vs. Time
%   4.  Attitude (Pitch/Roll/Heading) vs. Time
%   (RF Performance - if RF module enabled and data available)
%   5.  RF Link Status Timeline (OK/Fail/Skip indication per link)
%   6.  Received Power (dBm) vs. Time (per calculated link) + Sensitivity lines
%   7.  SNR (dB) vs. Time (per calculated link)
%   8.  Doppler Shift (Hz) vs. Time (per calculated link)
%   9.  SNR vs. Range Scatter Plot (per calculated link)
%   10. Shannon Capacity (Mbps) vs. Time (per calculated link)
%   11. Link Availability Summary (% time OK per calculated link)
%   12. Distance (km) & Rx Power (dBm) vs. Time (Combined plot, one per receiver)
%
% Usage:
%   Called from `main_simulation.m`:
%   `visualization_model(config, tspi_data, processed_rf_data, terrain, simulation_metadata);`
%
% Inputs:
%   config              - Main configuration struct from `config_settings.m`.
%   tspi_data           - NxM matrix of TSPI data logged during simulation.
%   processed_rf_data   - Structure containing processed time series RF data,
%                         organized by TxID -> RxID -> Parameter. Handles 'SKIP' status.
%   terrain             - Terrain model object (from `terrain_model.m`). Contains `data_loaded` flag.
%   simulation_metadata - Metadata struct containing `tspi_headers`, etc.
%
% Dependencies:
%   - `cyberpunk_theme.m` (if cyberpunk style selected).
%   - `snapshot_generator.m` (for saving plots).
%   - `earth_model.m` (for waypoint arrow calculations).
%   - `set_figure_style_helper_local.m` (external helper).
%   - `set_axes_style_helper_local.m` (external helper).
%   - `plot_terrain_helper_local.m` (local helper).      % <-- Clarified Dependency
%   - `apply_theme_defaults_local.m` (local helper).     % <-- Clarified Dependency
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-05] - Applied Code Analyzer fixes (MEXCEP, NASGU, AGROW)
% =========================================================================
function visualization_model(config, tspi_data, processed_rf_data, terrain, simulation_metadata) %

    fprintf('--- Generating Post-Simulation Plots ---\n'); %

    % --- Input Checks & Basic Setup ---
    if isempty(tspi_data) %
        warning('VisualizationModel:InputWarn','TSPI data is empty, cannot generate plots.'); % Added ID %
        return; %
    end %
    % Check if the processed RF data structure seems valid (has transmitter fields)
    rf_data_available = isstruct(processed_rf_data) && ~isempty(fieldnames(processed_rf_data)); %
    if ~rf_data_available %
        fprintf('Processed RF data is empty or invalid. RF-related plots will be skipped.\n'); %
    end %

    % Extract Time vector (assume first column of TSPI)
    time = tspi_data(:, 1); %
    num_steps = length(time); %
    if num_steps < 2 %
        warning('VisualizationModel:InputWarn','Need at least 2 time steps in TSPI data to generate meaningful plots.'); % Added ID %
        return; %
    end %
    % Estimate time step from first two points if needed later - Removed unused dt assignment (NASGU Fix line 93)

    % --- Constants and Unit Conversions ---
    FT2M = 0.3048; M2FT = 1 / FT2M; %
    KTS2MPS = 0.514444; MPS2KTS = 1 / KTS2MPS; %
    DEG2RAD = pi/180; RAD2DEG = 180/pi; %

    % --- Extract Key TSPI Columns using Headers for Robustness ---
    hdrs = lower(simulation_metadata.tspi_headers); % Use lowercase headers %
    % Find column indices; result is empty if header not found
    lat_col   = find(strcmp(hdrs, 'lat_deg'), 1); %
    lon_col   = find(strcmp(hdrs, 'lon_deg'), 1); %
    alt_col   = find(strcmp(hdrs, 'alt_m_msl'), 1); %
    hdg_col   = find(strcmp(hdrs, 'heading_deg'), 1); %
    pitch_col = find(strcmp(hdrs, 'pitch_deg'), 1); %
    roll_col  = find(strcmp(hdrs, 'roll_deg'), 1); %
    tas_col   = find(strcmp(hdrs, 'tas_mps'), 1); %
    gs_col    = find(strcmp(hdrs, 'groundspeed_mps'), 1); %
    vs_col    = find(strcmp(hdrs, 'verticalspeed_mps'), 1); %
    % Check if essential columns for plotting were found
    if isempty(lat_col) || isempty(lon_col) || isempty(alt_col) %
        error('VisualizationModel:InputError','Could not find required Latitude/Longitude/Altitude columns in TSPI data based on provided headers. Cannot generate plots.'); % Added ID %
    end %
    % Extract data vectors using the found indices
    lat_deg = tspi_data(:, lat_col); %
    lon_deg = tspi_data(:, lon_col); %
    alt_m   = tspi_data(:, alt_col); %
    alt_ft  = alt_m * M2FT; % Convert altitude to feet for display %

    % --- Apply Plot Theme ---
    theme = struct(); % Initialize empty theme struct %
    is_cyberpunk = strcmpi(config.visualization.PLOT_STYLE, 'cyberpunk'); %
    if is_cyberpunk %
        try %
            theme = cyberpunk_theme(); % Load theme settings from file %
            fprintf('Applying Cyberpunk plot theme.\n'); %
        catch ME_theme %
             % *** FIXED MEXCEP *** %
             warning(ME_theme.identifier, 'Failed to load cyberpunk theme from "%s": %s. Using standard MATLAB plots.', config.visualization.CYBERPUNK_THEME_FILE, ME_theme.message); %
             is_cyberpunk = false; % Revert to standard if theme loading fails %
        end %
    end %
    % Apply default theme values for robustness (handles standard style or incomplete theme)
    theme = apply_theme_defaults_local(theme); % Use local helper function %
    num_line_colors = length(theme.lineColors); % Get how many colors are defined in the theme %

    % --- Common Plotting Styles ---
    % Define common styles to apply to plots for consistency
    plot_styles = {'LineWidth', theme.lineWidth, 'MarkerSize', theme.markerSize, ... %
                   'FontName', theme.fontName, 'FontSize', theme.fontSize}; %

    % --- Initialize Figure Counter ---
    next_fig_num = 1; % Start with figure number 1 %

    % =====================================================================
    % --- Plot 1: 3D Flight Trajectory ---
    % =====================================================================
    try %
        fig_h = figure(next_fig_num); clf; % Create/clear figure %
        set_figure_style_helper_local(fig_h, theme, is_cyberpunk); % Apply figure style (CALLS EXTERNAL HELPER) %
        set(fig_h, 'Name', '3D Trajectory'); % Set window title %

        ax = axes('Parent', fig_h); % Create axes %
        hold(ax, 'on'); grid(ax, 'on'); view(ax, 3); % 3D view, hold for multiple plots %
        set_axes_style_helper_local(ax, theme, is_cyberpunk); % Apply axes style (CALLS EXTERNAL HELPER) %

        % --- Plot Elements ---
        % Trajectory line
        plot3(ax, lon_deg, lat_deg, alt_ft, '-', plot_styles{1:2}, 'Color', theme.lineColors{1}); %

        % Start/End Markers
        plot3(ax, lon_deg(1), lat_deg(1), alt_ft(1), 'o', 'MarkerSize', theme.markerSize+2, 'Color', theme.markerColorStart, 'MarkerFaceColor', theme.markerColorStart); % Start %
        plot3(ax, lon_deg(end), lat_deg(end), alt_ft(end), 's', 'MarkerSize', theme.markerSize+2, 'Color', theme.markerColorEnd, 'MarkerFaceColor', theme.markerColorEnd); % End %

        % Waypoints (Markers and Labels)
        wp = config.flight.waypoints; %
        wp_alt_ft = wp(:,3); 
        plot3(ax, wp(:,2), wp(:,1), wp_alt_ft, '^', plot_styles{3:4}, 'Color', theme.markerColorWaypoint, 'MarkerFaceColor', theme.markerColorWaypoint); %
        for i=1:size(wp,1), text(ax, wp(i,2), wp(i,1), wp_alt_ft(i), sprintf(' WP%d', i), 'Color', theme.textColor, 'FontSize', theme.fontSize-1, 'FontName', theme.fontName, 'VerticalAlignment', 'bottom'); end %

        % Waypoint Heading Arrows (Patch 8)
        if size(wp, 2) >= 5 % Check if TargetHeading column exists %
            arrow_scale_m = config.visualization.WAYPOINT_ARROW_SCALE; % Length in meters %
            earth_model_temp = earth_model(config.earth.model); % Need earth model for 'move' %
            for i = 1:size(wp, 1) %
                 wp_lat = wp(i,1); wp_lon = wp(i,2); wp_alt_ft_i = wp_alt_ft(i); wp_hdg = wp(i,5); %
                 if ~isnan(wp_hdg) % Only plot if heading command exists %
                     [arrow_lat, arrow_lon] = earth_model_temp.move(wp_lat, wp_lon, arrow_scale_m, wp_hdg); % Calculate arrow end point %
                     quiver3(ax, wp_lon, wp_lat, wp_alt_ft_i, arrow_lon-wp_lon, arrow_lat-wp_lat, 0, 0, ... % Plot arrow vector %
                             'Color', theme.markerColorWaypoint, 'LineWidth', 1, 'MaxHeadSize', 0.5); %
                 end %
            end %
        end %

        % Ground Stations (Markers and Labels)
        if isfield(config.rf, 'ground_stations') && ~isempty(config.rf.ground_stations) %
            for i = 1:length(config.rf.ground_stations) %
                gs = config.rf.ground_stations{i}; gs_alt_ft = gs.alt_m_msl * M2FT; %
                plot3(ax, gs.lon_deg, gs.lat_deg, gs_alt_ft, 's', plot_styles{3:4}, 'Color', theme.markerColorGroundStation, 'MarkerFaceColor', theme.markerColorGroundStation); %
                text(ax, gs.lon_deg, gs.lat_deg, gs_alt_ft, sprintf(' %s', gs.id), 'Color', theme.textColor, 'FontSize', theme.fontSize-1, 'FontName', theme.fontName, 'VerticalAlignment', 'bottom'); %
            end %
        end %

        % Plot Terrain Surface (Optional)
        if terrain.data_loaded && config.visualization.ENABLE_TERRAIN_VIEW %
            fprintf('  Plotting terrain surface for 3D Trajectory...\n'); %
            try plot_terrain_helper_local(ax, terrain, xlim(ax), ylim(ax), theme); % Use local helper %
            catch ME_terrain_plot % *** FIXED MEXCEP *** %
                 warning(ME_terrain_plot.identifier, 'Failed to plot terrain in 3D view: %s', ME_terrain_plot.message); %
            end %
        end %

        % Load Airspace Boundary
        load('AirspaceShape.mat'); % Loads shapeData (1×32 cell array)

        for i = 1:numel(shapeData)
            entry = shapeData{i};  % ✅ extract struct from cell

            % Extract name
            try
                name = string(entry.Name);  % name is likely a string or char
            catch
                name = "Unnamed";
            end

            % Extract lat/lon
            lat = entry.Latitudes;
            lon = entry.Longitudes;

            % Unwrap from cell if needed
            if iscell(lat), lat = lat{1}; end
            if iscell(lon), lon = lon{1}; end

            if isempty(lat) || isempty(lon), continue; end

            % Filter NaNs
            valid_idx = ~isnan(lat) & ~isnan(lon);
            if nnz(valid_idx) < 3, continue; end

            % Elevation to float above terrain
            elev = max(alt_ft(:), [], 'omitnan') + 100;

            % Plot airspace outline
            plot3(lon(valid_idx), lat(valid_idx), elev * ones(size(lat(valid_idx))), ...
                'r-', 'LineWidth', 1.5);

        end

% [LAT_sub, LON_sub] = terrain.ref_obj.geographicGrid();
% 
% % --- Fix axis scaling to match real-world distance ---
% lat_avg = mean(LAT_sub(:), 'omitnan');
% meters_per_deg_lat = 111320;
% meters_per_deg_lon = 111320 * cosd(lat_avg);
% 
% z_scale_factor = 1;  % <- 1 = true vertical scale, >1 exaggerates terrain
% 
% % Set axis aspect ratio so distances scale evenly
% % daspect(ax, [1/meters_per_deg_lon, 1/meters_per_deg_lat, 1/z_scale_factor]);  % X:Y:Z scale

        % --- Finalize Plot 1 ---
        hold(ax, 'off'); %
        title('3D Flight Trajectory', 'Color', theme.titleColor, 'FontName', theme.fontName, 'FontSize', theme.fontSize+1); %
        xlabel('Longitude (deg)', 'Color', theme.labelColor); ylabel('Latitude (deg)', 'Color', theme.labelColor); zlabel('Altitude MSL (ft)', 'Color', theme.labelColor); %
        %axis tight; % Fit axes limits to data %
        view(30, 30); % Set initial 3D viewpoint %
        %axis vis3d; % Keep aspect ratio fixed during rotation %

        % Save snapshot using utility function (checks export flag)
        snapshot_generator(fig_h, 'trajectory_3d', config, next_fig_num); %
        next_fig_num = next_fig_num + 1; % Increment figure counter %

    catch ME_plot % Catch errors during plot generation %
        % *** FIXED MEXCEP *** %
        warning(ME_plot.identifier, 'Failed to generate Plot 1 (3D Trajectory): %s', ME_plot.message); %
        if exist('fig_h','var') && ishandle(fig_h), try close(fig_h); catch; end; end % Close faulty figure %
        next_fig_num = next_fig_num + 1; % Ensure counter increments even on failure %
    end %

    % =====================================================================
    % --- Plot 2: Altitude vs. Time ---
    % =====================================================================
    try %
        fig_h = figure(next_fig_num); clf; set_figure_style_helper_local(fig_h, theme, is_cyberpunk); % CALLS EXTERNAL HELPER %
        set(fig_h, 'Name', 'Altitude vs Time'); ax = axes('Parent', fig_h); %
        hold(ax,'on'); grid(ax,'on'); set_axes_style_helper_local(ax, theme, is_cyberpunk); % CALLS EXTERNAL HELPER %

        plot(ax, time, alt_ft, plot_styles{1:2}, 'Color', theme.lineColors{2}); % Plot Alt in feet %

        hold(ax,'off'); %
        title('Altitude vs. Time', 'Color', theme.titleColor, 'FontName', theme.fontName); %
        xlabel('Time (s)', 'Color', theme.labelColor); ylabel('Altitude MSL (ft)', 'Color', theme.labelColor); %
        ylimits = ylim(ax); ylim(ax, [0, max(ylimits(2)*1.1, min(alt_ft)*0.9)]); % Ensure y starts near min alt, slightly above max %

        snapshot_generator(fig_h, 'altitude_vs_time', config, next_fig_num); %
        next_fig_num = next_fig_num + 1; %
     catch ME_plot %
         % *** FIXED MEXCEP *** %
         warning(ME_plot.identifier, 'Failed to generate Plot 2 (Altitude): %s', ME_plot.message); %
         if exist('fig_h','var') && ishandle(fig_h), try close(fig_h); catch; end; end %
         next_fig_num = next_fig_num + 1; %
     end %

    % =====================================================================
    % --- Plot 3: Speed vs. Time ---
    % =====================================================================
     try %
         fig_h = figure(next_fig_num); clf; set_figure_style_helper_local(fig_h, theme, is_cyberpunk); % CALLS EXTERNAL HELPER %
         set(fig_h, 'Name', 'Speed vs Time'); ax = axes('Parent', fig_h); %
         hold(ax,'on'); grid(ax,'on'); set_axes_style_helper_local(ax, theme, is_cyberpunk); % CALLS EXTERNAL HELPER %
         lgd_items = {}; legend_handles = []; %

         % Plot True Airspeed (TAS) in knots if available
         if ~isempty(tas_col) %
             h_tas = plot(ax, time, tspi_data(:, tas_col) * MPS2KTS, plot_styles{1:2}, 'Color', theme.lineColors{3}); %
             lgd_items{end+1} = 'True Airspeed (TAS)'; legend_handles(end+1) = h_tas; %
         end %
         % Plot Ground Speed in knots if available
         if ~isempty(gs_col) %
             h_gs = plot(ax, time, tspi_data(:, gs_col) * MPS2KTS, plot_styles{1:2}, 'Color', theme.lineColors{4}, 'LineStyle', '--'); %
             lgd_items{end+1} = 'Ground Speed'; legend_handles(end+1) = h_gs; %
         end %

         hold(ax,'off'); %
         title('Speed vs. Time', 'Color', theme.titleColor, 'FontName', theme.fontName); %
         xlabel('Time (s)', 'Color', theme.labelColor); ylabel('Speed (knots)', 'Color', theme.labelColor); %
         if ~isempty(legend_handles) % Add legend only if plots were made %
             lgd = legend(ax, legend_handles, lgd_items, 'Location', 'best'); %
             set(lgd, 'TextColor', theme.textColor, 'Color', theme.axesColor, 'EdgeColor', theme.gridColor); % Style legend %
         end %

         snapshot_generator(fig_h, 'speed_vs_time', config, next_fig_num); %
         next_fig_num = next_fig_num + 1; %
      catch ME_plot %
          % *** FIXED MEXCEP *** %
          warning(ME_plot.identifier, 'Failed to generate Plot 3 (Speed): %s', ME_plot.message); %
          if exist('fig_h','var') && ishandle(fig_h), try close(fig_h); catch; end; end %
          next_fig_num = next_fig_num + 1; %
      end %

    % =====================================================================
    % --- Plot 4: Attitude vs. Time ---
    % =====================================================================
      try %
          fig_h = figure(next_fig_num); clf; set_figure_style_helper_local(fig_h, theme, is_cyberpunk); % CALLS EXTERNAL HELPER %
          set(fig_h, 'Name', 'Attitude vs Time'); axes_att = gobjects(3,1); % Array for subplot axes handles %

          % Subplot 1: Pitch
          axes_att(1) = subplot(3,1,1, 'Parent', fig_h); hold(axes_att(1),'on'); grid on; set_axes_style_helper_local(axes_att(1), theme, is_cyberpunk); % CALLS EXTERNAL HELPER %
          if ~isempty(pitch_col) %
              plot(axes_att(1), time, tspi_data(:, pitch_col), plot_styles{1:2}, 'Color', theme.lineColors{5}); %
          end %
          hold(axes_att(1),'off'); ylabel('Pitch (deg)', 'Color', theme.labelColor); title('Attitude vs. Time', 'Color', theme.titleColor); %

          % Subplot 2: Roll
          axes_att(2) = subplot(3,1,2, 'Parent', fig_h); hold(axes_att(2),'on'); grid on; set_axes_style_helper_local(axes_att(2), theme, is_cyberpunk); % CALLS EXTERNAL HELPER %
          if ~isempty(roll_col) %
              plot(axes_att(2), time, tspi_data(:, roll_col), plot_styles{1:2}, 'Color', theme.lineColors{6}); %
          end %
          hold(axes_att(2),'off'); ylabel('Roll (deg)', 'Color', theme.labelColor); %

          % Subplot 3: Heading
          axes_att(3) = subplot(3,1,3, 'Parent', fig_h); hold(axes_att(3),'on'); grid on; set_axes_style_helper_local(axes_att(3), theme, is_cyberpunk); % CALLS EXTERNAL HELPER %
           if ~isempty(hdg_col) %
               % Unwrap heading angle from [0, 360) to continuous degrees for plotting
               heading_unwrapped_rad = unwrap(tspi_data(:, hdg_col)*DEG2RAD); %
               plot(axes_att(3), time, heading_unwrapped_rad*RAD2DEG, plot_styles{1:2}, 'Color', theme.lineColors{1}); %
           end %
          hold(axes_att(3),'off'); xlabel('Time (s)', 'Color', theme.labelColor); ylabel('Heading (deg)', 'Color', theme.labelColor); %

          % Link X-axes of subplots
          linkaxes(axes_att, 'x'); %
          xlim(axes_att(1), [time(1) time(end)]); % Ensure x-limits match data %
          snapshot_generator(fig_h, 'attitude_vs_time', config, next_fig_num); %
          next_fig_num = next_fig_num + 1; %
      catch ME_plot %
            % *** FIXED MEXCEP *** %
            warning(ME_plot.identifier, 'Failed to generate Plot 4 (Attitude): %s', ME_plot.message); %
            if exist('fig_h','var') && ishandle(fig_h), try close(fig_h); catch; end; end %
            next_fig_num = next_fig_num + 1; %
      end %


    % =====================================================================
    % --- RF Performance Plots (Generated only if RF data is available) ---
    % =====================================================================
    if rf_data_available %
        tx_ids = fieldnames(processed_rf_data); % Get transmitter IDs from processed data struct %
        link_counter      = 0; % Counter for Y-axis positioning on status plot %
        num_valid_links   = 0; % Counter for links that are actually plotted (not skipped) %
        figs_rf           = struct(); % Store figure handles for RF plots %
        lgd_handles       = struct(); % Store handles and labels for legends for each plot type %

        % --- Loop through each Transmitter ---
        for i_tx = 1:length(tx_ids) %
            tx_id = tx_ids{i_tx}; %
            % Check if Tx ID exists as a field and contains receiver data structs
             if ~isfield(processed_rf_data, tx_id) || ~isstruct(processed_rf_data.(tx_id)) %
                 warning('VisualizationModel:RFDataWarn','Processed RF data structure malformed for Tx: %s. Skipping.', tx_id); % Added ID %
                 continue; %
             end %
             rx_ids = fieldnames(processed_rf_data.(tx_id)); %

             % --- Loop through each Receiver for this Transmitter ---
             for i_rx = 1:length(rx_ids) %
                 rx_id = rx_ids{i_rx}; %
                 % Create a user-friendly label for legends etc. (replace underscores)
                 link_label = sprintf('%s > %s', strrep(tx_id,'_','-'), strrep(rx_id,'_','-')); %
                 link_counter = link_counter + 1; % Increment master counter for Y-positioning %

                 % --- Check if this link has valid data or was skipped ---
                 % Verify the link data structure exists
                 if ~isfield(processed_rf_data.(tx_id), rx_id) || ~isstruct(processed_rf_data.(tx_id).(rx_id)) %
                     fprintf('Skipping plots for link %s (Data missing/invalid in processed struct)\n', link_label); %
                     continue; % Skip to next link %
                 end %
                 link_data = processed_rf_data.(tx_id).(rx_id); % Get data structure for this link %

                 % Check if the link was skipped during calculation OR has no valid numeric data
                 % Assumes 'status' cell array and 'snr_db' numeric array exist
                 is_skipped_link  = any(strcmp(link_data.status, 'SKIP')); %
                 has_numeric_data = isfield(link_data,'snr_db') && ~all(isnan(link_data.snr_db)); % Example check %

                 % --- Plot 5: RF Link Status Timeline (Update for all links, even skipped) ---
                 % Initialize figure on first loop entry
                 if link_counter == 1 %
                     figs_rf.Status = figure(next_fig_num); clf; set(figs_rf.Status,'Name','Link Status'); set_figure_style_helper_local(figs_rf.Status,theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     figs_rf.axStatus = axes('Parent', figs_rf.Status); hold(figs_rf.axStatus,'on'); grid on; set_axes_style_helper_local(figs_rf.axStatus,theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     ylabel('Links','Color',theme.labelColor); xlabel('Time(s)','Color',theme.labelColor); title('RF Link Status Timeline','Color',theme.titleColor); %
                     set(figs_rf.axStatus, 'YColor', 'none'); % Hide Y-axis line/ticks, just use labels later %
                     lgd_handles.Status.ypos   = []; % Y positions for labels %
                     lgd_handles.Status.labels = {}; % Link labels for Y-axis %
                     next_fig_num = next_fig_num + 1; %
                 end %
                 % Determine color based on status
                 status_colors = zeros(num_steps, 3); % Initialize color matrix [Mx3] %
                 status_ok_mask   = strcmp(link_data.status, 'OK'); %
                 status_fail_mask = strcmp(link_data.status, 'FAIL'); %
                 status_skip_mask = strcmp(link_data.status, 'SKIP'); %
                 % Handle cases where status might be empty or different (assign unknown color)
                 status_other_mask = ~(status_ok_mask | status_fail_mask | status_skip_mask); %
                 status_colors(status_ok_mask,:)   = repmat(theme.statusOkColor, sum(status_ok_mask), 1); %
                 status_colors(status_fail_mask,:)  = repmat(theme.statusFailColor, sum(status_fail_mask), 1); %
                 status_colors(status_skip_mask,:) = repmat(theme.statusUnknownColor*0.7, sum(status_skip_mask), 1); % Dimmed skip color %
                 status_colors(status_other_mask,:) = repmat(theme.statusUnknownColor, sum(status_other_mask), 1); % Use standard unknown color %
                 % Draw colored patches for this link's status over time
                 y_pos = link_counter; % Use master counter for Y position %
                 bar_height = theme.LINK_STATUS_PLOT_HEIGHT; %
                 % Define patch X coordinates using time_diff (robust to variable dt)
                 time_diff = diff(time); if isempty(time_diff), time_diff = 1; else time_diff = [time_diff(1); time_diff]; end % Estimate dt per step %
                 patch_x = [time-time_diff/2, time+time_diff/2, time+time_diff/2, time-time_diff/2]'; % X coords [4xN] %

                 patch_y = repmat([y_pos-bar_height/2, y_pos-bar_height/2, y_pos+bar_height/2, y_pos+bar_height/2], num_steps, 1)'; % Y coords [4xN] %
                 % Use patch with FaceVertexCData for efficiency (colors each patch individually)
                 patch(figs_rf.axStatus, patch_x, patch_y, permute(status_colors,[1 3 2]), 'FaceColor','flat', 'EdgeColor','none'); % Color by row vector corresponding to patch %

                 lgd_handles.Status.ypos(end+1)   = y_pos; % Store Y position for label %
                 lgd_handles.Status.labels{end+1} = link_label; % Store label %

                 % --- Skip remaining plots for this link if invalid ---
                 if is_skipped_link || ~has_numeric_data %
                     if ~is_skipped_link % Log if skipped due to missing data rather than explicit 'SKIP' status %
                         fprintf('Skipping numerical plots for link %s (No valid numeric data found)\n', link_label); %
                     end %
                     continue; % <<< Skip to the next Link >>> %
                 end %

                 % --- Link has valid data, proceed with numerical plots ---
                 num_valid_links = num_valid_links + 1; % Increment counter of links actually plotted %

                 % --- Calculate color index for this valid link ---
                 if num_valid_links > 0 % Ensure num_valid_links is at least 1 before modulo %
                     color_index = mod(num_valid_links - 1, num_line_colors) + 1; %
                 else % Handle the very first valid link case %
                     color_index = 1; %
                 end %
                 current_line_color = theme.lineColors{color_index}; %

                 % --- Plot 6: Received Power vs. Time ---
                 plot_title = 'Received Power vs Time'; plot_tag = 'RxPwr'; plot_ylabel = 'Rx Power (dBm)'; %
                 if ~isfield(figs_rf, plot_tag) % Initialize figure/axes on first valid link for this plot type %
                     figs_rf.(plot_tag) = figure(next_fig_num); clf; set(figs_rf.(plot_tag),'Name',plot_title); set_figure_style_helper_local(figs_rf.(plot_tag),theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     figs_rf.(['ax' plot_tag]) = axes('Parent',figs_rf.(plot_tag)); hold(figs_rf.(['ax' plot_tag]),'on'); grid on; set_axes_style_helper_local(figs_rf.(['ax' plot_tag]),theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     ylabel(figs_rf.(['ax' plot_tag]), plot_ylabel,'Color',theme.labelColor); xlabel(figs_rf.(['ax' plot_tag]), 'Time(s)','Color',theme.labelColor); title(figs_rf.(['ax' plot_tag]), plot_title,'Color',theme.titleColor); %
                     lgd_handles.(plot_tag).h = []; lgd_handles.(plot_tag).l = {}; lgd_handles.(plot_tag).sens_plotted = {}; % Init legend stores %
                     next_fig_num = next_fig_num + 1; %
                 end %
                 h_plot = plot(figs_rf.(['ax' plot_tag]), time, link_data.received_power_dbm, plot_styles{1:2}, 'Color', current_line_color); % Plot data %
                 lgd_handles.(plot_tag).h(end+1) = h_plot; lgd_handles.(plot_tag).l{end+1} = link_label; % Store handle/label %
                 % Plot sensitivity line for this receiver (only once per unique Rx)
                 sens_label = sprintf('%s Sens.', rx_id); %
                 if ~ismember(sens_label, lgd_handles.(plot_tag).sens_plotted) && isfield(link_data,'rx_sensitivity_dbm') && ~isempty(link_data.rx_sensitivity_dbm) && isfinite(link_data.rx_sensitivity_dbm(1)) %
                      plot(figs_rf.(['ax' plot_tag]), time, repmat(link_data.rx_sensitivity_dbm(1), num_steps, 1), ':', 'Color', current_line_color, 'LineWidth', theme.lineWidth-0.5); % Threshold line %
                      h_sens = plot(NaN,NaN,':', 'Color', current_line_color, 'LineWidth', theme.lineWidth-0.5); %#ok<NASGU> % Dummy plot for legend; NASGU ok %
                      lgd_handles.(plot_tag).h(end+1) = h_sens; lgd_handles.(plot_tag).l{end+1} = sens_label; lgd_handles.(plot_tag).sens_plotted{end+1} = sens_label; % Mark sensitivity as plotted for this Rx %
                 end %

                 % --- Plot 7: SNR vs Time ---
                 plot_title = 'SNR vs Time'; plot_tag = 'SNR'; plot_ylabel = 'SNR (dB)'; %
                 if ~isfield(figs_rf, plot_tag) %
                     figs_rf.(plot_tag)=figure(next_fig_num);clf;set(figs_rf.(plot_tag),'Name',plot_title);set_figure_style_helper_local(figs_rf.(plot_tag),theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     figs_rf.(['ax' plot_tag])=axes('Parent',figs_rf.(plot_tag));hold(figs_rf.(['ax' plot_tag]),'on');grid on;set_axes_style_helper_local(figs_rf.(['ax' plot_tag]),theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     ylabel(figs_rf.(['ax' plot_tag]), plot_ylabel,'Color',theme.labelColor); xlabel(figs_rf.(['ax' plot_tag]), 'Time(s)','Color',theme.labelColor); title(figs_rf.(['ax' plot_tag]), plot_title,'Color',theme.titleColor); %
                     lgd_handles.(plot_tag).h=[];lgd_handles.(plot_tag).l={};next_fig_num=next_fig_num+1; %
                 end %
                 h=plot(figs_rf.(['ax' plot_tag]),time,link_data.snr_db,plot_styles{1:2},'Color',current_line_color); %#ok<NASGU> % NASGU ok, handle stored %
                 lgd_handles.(plot_tag).h(end+1)=h;lgd_handles.(plot_tag).l{end+1}=link_label; %

                 % --- Plot 8: Doppler vs Time ---
                 plot_title = 'Doppler Shift vs Time'; plot_tag = 'Doppler'; plot_ylabel = 'Doppler Shift (Hz)'; %
                 if ~isfield(figs_rf, plot_tag) %
                     figs_rf.(plot_tag)=figure(next_fig_num);clf;set(figs_rf.(plot_tag),'Name',plot_title);set_figure_style_helper_local(figs_rf.(plot_tag),theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     figs_rf.(['ax' plot_tag])=axes('Parent',figs_rf.(plot_tag));hold(figs_rf.(['ax' plot_tag]),'on');grid on;set_axes_style_helper_local(figs_rf.(['ax' plot_tag]),theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     ylabel(figs_rf.(['ax' plot_tag]), plot_ylabel,'Color',theme.labelColor); xlabel(figs_rf.(['ax' plot_tag]), 'Time(s)','Color',theme.labelColor); title(figs_rf.(['ax' plot_tag]), plot_title,'Color',theme.titleColor); %
                     lgd_handles.(plot_tag).h=[];lgd_handles.(plot_tag).l={};next_fig_num=next_fig_num+1; %
                 end %
                 h=plot(figs_rf.(['ax' plot_tag]),time,link_data.doppler_shift_hz,plot_styles{1:2},'Color',current_line_color); %#ok<NASGU> % NASGU ok, handle stored %
                 lgd_handles.(plot_tag).h(end+1)=h;lgd_handles.(plot_tag).l{end+1}=link_label; %

                 % --- Plot 9: SNR vs Range Scatter ---
                 plot_title = 'SNR vs Range'; plot_tag = 'SNRRange'; plot_ylabel = 'SNR (dB)'; plot_xlabel = 'Range (km)'; %
                 if ~isfield(figs_rf, plot_tag) %
                     figs_rf.(plot_tag)=figure(next_fig_num);clf;set(figs_rf.(plot_tag),'Name',plot_title);set_figure_style_helper_local(figs_rf.(plot_tag),theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     figs_rf.(['ax' plot_tag])=axes('Parent',figs_rf.(plot_tag));hold(figs_rf.(['ax' plot_tag]),'on');grid on;set_axes_style_helper_local(figs_rf.(['ax' plot_tag]),theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     ylabel(figs_rf.(['ax' plot_tag]), plot_ylabel,'Color',theme.labelColor); xlabel(figs_rf.(['ax' plot_tag]), plot_xlabel,'Color',theme.labelColor); title(figs_rf.(['ax' plot_tag]), plot_title,'Color',theme.titleColor); %
                     lgd_handles.(plot_tag).h=[];lgd_handles.(plot_tag).l={};next_fig_num=next_fig_num+1; %
                 end %
                 h=scatter(figs_rf.(['ax' plot_tag]),link_data.range_m/1000,link_data.snr_db,theme.markerSize^2,current_line_color,'filled','MarkerFaceAlpha',0.6); %#ok<NASGU> % NASGU ok, handle stored %
                 lgd_handles.(plot_tag).h(end+1)=h;lgd_handles.(plot_tag).l{end+1}=link_label; %

                 % --- Plot 10: Capacity vs Time ---
                 plot_title = 'Channel Capacity vs Time'; plot_tag = 'Capacity'; plot_ylabel = 'Capacity (Mbps)'; %
                 if ~isfield(figs_rf, plot_tag) %
                     figs_rf.(plot_tag)=figure(next_fig_num);clf;set(figs_rf.(plot_tag),'Name',plot_title);set_figure_style_helper_local(figs_rf.(plot_tag),theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     figs_rf.(['ax' plot_tag])=axes('Parent',figs_rf.(plot_tag));hold(figs_rf.(['ax' plot_tag]),'on');grid on;set_axes_style_helper_local(figs_rf.(['ax' plot_tag]),theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
                     ylabel(figs_rf.(['ax' plot_tag]), plot_ylabel,'Color',theme.labelColor); xlabel(figs_rf.(['ax' plot_tag]), 'Time(s)','Color',theme.labelColor); title(figs_rf.(['ax' plot_tag]), plot_title,'Color',theme.titleColor); %
                     lgd_handles.(plot_tag).h=[];lgd_handles.(plot_tag).l={};next_fig_num=next_fig_num+1; %
                 end %
                 h=plot(figs_rf.(['ax' plot_tag]),time,link_data.shannon_capacity_bps/1e6,plot_styles{1:2},'Color',current_line_color); %#ok<NASGU> % NASGU ok, handle stored %
                 lgd_handles.(plot_tag).h(end+1)=h;lgd_handles.(plot_tag).l{end+1}=link_label; %

                 % % --- Plot 12: Distance & Rx Power vs Time (Combined Y-Axes, One Figure Per Receiver) ---
                 % dist_pwr_figname = ['DistPwr_' rx_id]; % Unique field name for this receiver's plot figure/handles %
                 % if ~isfield(figs_rf, dist_pwr_figname) % Initialize figure only once per unique receiver %
                 %      figs_rf.(dist_pwr_figname) = figure(next_fig_num); clf; set(figs_rf.(dist_pwr_figname),'Name',['Dist/Pwr ' rx_id]); set_figure_style_helper_local(figs_rf.(dist_pwr_figname),theme,is_cyberpunk); next_fig_num=next_fig_num+1; % CALLS EXTERNAL HELPER %
                 %      % Create left Y-axis for Distance
                 %      ax_dist = axes('Parent', figs_rf.(dist_pwr_figname), 'YAxisLocation', 'left'); hold(ax_dist,'on'); grid on; set_axes_style_helper_local(ax_dist,theme,is_cyberpunk); ylabel(ax_dist,'Distance(km)','Color',theme.labelColor); xlabel(ax_dist,'Time(s)','Color',theme.labelColor); title(sprintf('Distance & Rx Power (%s)',strrep(rx_id,'_','-')),'Color',theme.titleColor); % CALLS EXTERNAL HELPER %
                 %      % Create right Y-axis for Power (overlay, transparent background)
                 %      ax_pwr = axes('Parent', figs_rf.(dist_pwr_figname), 'Position', get(ax_dist,'Position'), 'YAxisLocation', 'right', 'Color','none'); hold(ax_pwr,'on'); set_axes_style_helper_local(ax_pwr,theme,is_cyberpunk); set(ax_pwr,'XGrid','off','YGrid','off','XTick',[]); ylabel(ax_pwr,'Rx Power(dBm)','Color',theme.labelColor); % Turn off grid/ticks for overlay % CALLS EXTERNAL HELPER %
                 %      linkaxes([ax_dist, ax_pwr], 'x'); % Link X-axes %
                 %      figs_rf.(dist_pwr_figname).UserData.ax_dist = ax_dist; figs_rf.(dist_pwr_figname).UserData.ax_pwr = ax_pwr; % Store axes handles in figure UserData %
                 %      lgd_handles.(dist_pwr_figname).h = []; lgd_handles.(dist_pwr_figname).l = {}; lgd_handles.(dist_pwr_figname).sens_plotted = {}; % Init legend store %
                 % else % Figure already exists, get axes handles %
                 %      ax_dist = figs_rf.(dist_pwr_figname).UserData.ax_dist; %
                 %      ax_pwr = figs_rf.(dist_pwr_figname).UserData.ax_pwr; %
                 % end %
                 % % Plot distance on left axis
                 % axes(ax_dist); % Make left axes current; LAXES warning ok %
                 % h_dist = plot(ax_dist, time, link_data.range_m/1000, plot_styles{1:2}, 'Color', current_line_color, 'LineStyle', '-'); %#ok<NASGU> % NASGU warning for h_dist is ok %
                 % lgd_handles.(dist_pwr_figname).h(end+1)=h_dist; lgd_handles.(dist_pwr_figname).l{end+1}=sprintf('%s Dist',link_label); %
                 % set(ax_dist, 'YColor', current_line_color); % Color axis to match line? Or keep theme default? %
                 % 
                 % % Plot power on right axis
                 % axes(ax_pwr); % Make right axes current; LAXES warning ok %
                 % h_pwr = plot(ax_pwr, time, link_data.received_power_dbm, plot_styles{1:2}, 'Color', current_line_color, 'LineStyle', '--'); %#ok<NASGU> % NASGU warning for h_pwr is ok %
                 % lgd_handles.(dist_pwr_figname).h(end+1)=h_pwr; lgd_handles.(dist_pwr_figname).l{end+1}=sprintf('%s Pwr',link_label); %
                 % set(ax_pwr, 'YColor', current_line_color); % Color axis to match line? Or keep theme default? %
                 % 
                 % % Add sensitivity threshold line (related to right axis) - plot only once per Rx
                 % sens_label_dist = sprintf('%s Sens.', rx_id); %
                 % if ~ismember(sens_label_dist, lgd_handles.(dist_pwr_figname).sens_plotted) && isfield(link_data,'rx_sensitivity_dbm') && ~isempty(link_data.rx_sensitivity_dbm) && isfinite(link_data.rx_sensitivity_dbm(1)) %
                 %      plot(ax_pwr, time, repmat(link_data.rx_sensitivity_dbm(1), num_steps, 1), ':', 'Color', current_line_color, 'LineWidth', theme.lineWidth-0.5); %
                 %      h_sens_dist = plot(ax_pwr, NaN, NaN, ':', 'Color', current_line_color, 'LineWidth', theme.lineWidth-0.5); %#ok<NASGU> % Dummy for legend; NASGU warning ok %
                 %      lgd_handles.(dist_pwr_figname).h(end+1)=h_sens_dist; lgd_handles.(dist_pwr_figname).l{end+1}=sens_label_dist; %
                 %      lgd_handles.(dist_pwr_figname).sens_plotted{end+1} = sens_label_dist; % Mark as plotted for this Rx %
                 % end %
                 % % Reset current axes back to left maybe? Or doesn't matter now.
                 % axes(ax_dist); % LAXES warning ok %

                 % --- Plot 12: Distance & Rx Power vs Time (Combined Y-Axes, One Figure Per Receiver) ---
% --- Plot 12: Distance & Rx Power vs Time (Combined Y-Axes, One Figure Per Receiver) ---
dist_pwr_figname = ['DistPwr_' rx_id]; % Unique field name for this receiver's plot figure/handles %
if ~isfield(figs_rf, dist_pwr_figname) % Initialize figure only once per unique receiver %
figs_rf.(dist_pwr_figname) = figure(next_fig_num); clf; set(figs_rf.(dist_pwr_figname),'Name',['Dist/Pwr ' rx_id]); set_figure_style_helper_local(figs_rf.(dist_pwr_figname),theme,is_cyberpunk); next_fig_num=next_fig_num+1; % CALLS EXTERNAL HELPER %
% Create left Y-axis for Distance
ax_dist = axes('Parent', figs_rf.(dist_pwr_figname), 'YAxisLocation', 'left'); hold(ax_dist,'on'); grid on; set_axes_style_helper_local(ax_dist,theme,is_cyberpunk); ylabel(ax_dist,'Distance(km)','Color',theme.labelColor); xlabel(ax_dist,'Time(s)','Color',theme.labelColor); title(sprintf('Distance & Rx Power (%s)',strrep(rx_id,'_','-')),'Color',theme.titleColor); % CALLS EXTERNAL HELPER %
% Create right Y-axis for Power (overlay, transparent background)
ax_pwr = axes('Parent', figs_rf.(dist_pwr_figname), 'Position', get(ax_dist,'Position'), 'YAxisLocation', 'right', 'Color','none'); hold(ax_pwr,'on'); set_axes_style_helper_local(ax_pwr,theme,is_cyberpunk); set(ax_pwr,'XGrid','off','YGrid','off','XTick',[]); ylabel(ax_pwr,'Rx Power(dBm)','Color',theme.labelColor); % Turn off grid/ticks for overlay % CALLS EXTERNAL HELPER %
linkaxes([ax_dist, ax_pwr], 'x'); % Link X-axes %
% Store axes handles in UserData for later retrieval
figs_rf.(dist_pwr_figname).UserData.ax_dist = ax_dist;
figs_rf.(dist_pwr_figname).UserData.ax_pwr = ax_pwr;
% Initialize legend handles structure for this figure
lgd_handles.(dist_pwr_figname).h = [];
lgd_handles.(dist_pwr_figname).l = {};
lgd_handles.(dist_pwr_figname).sens_plotted = {};
else % Figure already exists, get axes handles %
ax_dist = figs_rf.(dist_pwr_figname).UserData.ax_dist;
ax_pwr = figs_rf.(dist_pwr_figname).UserData.ax_pwr;
end %

% --- Prepare data for plotting ---
range_km_data = link_data.range_m / 1000;
power_dbm_data = link_data.received_power_dbm;
sensitivity_dbm_data = link_data.rx_sensitivity_dbm(1); % Assuming sensitivity is constant

% --- Plot distance on left axis ---
% Plotting directly onto the axes handle is cleaner
h_dist = plot(ax_dist, time, range_km_data, plot_styles{1:2}, 'Color', current_line_color, 'LineStyle', '-');
lgd_handles.(dist_pwr_figname).h(end+1)=h_dist; lgd_handles.(dist_pwr_figname).l{end+1}=sprintf('%s Dist',link_label);

% --- Plot power on right axis ---
h_pwr = plot(ax_pwr, time, power_dbm_data, plot_styles{1:2}, 'Color', current_line_color, 'LineStyle', '--');
lgd_handles.(dist_pwr_figname).h(end+1)=h_pwr; lgd_handles.(dist_pwr_figname).l{end+1}=sprintf('%s Pwr',link_label);

% --- Add sensitivity threshold line (related to right axis) - plot only once per Rx ---
sens_label_dist = sprintf('%s Sens.', rx_id);
if ~ismember(sens_label_dist, lgd_handles.(dist_pwr_figname).sens_plotted) && isfield(link_data,'rx_sensitivity_dbm') && ~isempty(link_data.rx_sensitivity_dbm) && isfinite(sensitivity_dbm_data)
plot(ax_pwr, time, repmat(sensitivity_dbm_data, num_steps, 1), ':', 'Color', current_line_color, 'LineWidth', theme.lineWidth-0.5);
% Create dummy plot handle for legend entry
h_sens_dist = plot(ax_pwr, NaN, NaN, ':', 'Color', current_line_color, 'LineWidth', theme.lineWidth-0.5);
lgd_handles.(dist_pwr_figname).h(end+1)=h_sens_dist;
lgd_handles.(dist_pwr_figname).l{end+1}=sens_label_dist;
lgd_handles.(dist_pwr_figname).sens_plotted{end+1} = sens_label_dist; % Mark as plotted for this Rx
end
% No need to switch axes back and forth

% --- Finalize Dist/Pwr Plots (Set Limits, Legends) AFTER Tx/Rx loops ---
dist_pwr_plot_names = fieldnames(figs_rf); % Get all figure field names
dist_pwr_plot_names = dist_pwr_plot_names(startsWith(dist_pwr_plot_names, 'DistPwr_')); % Filter for only DistPwr plots

for i_dp = 1:length(dist_pwr_plot_names)
plot_name = dist_pwr_plot_names{i_dp};
fig_h = figs_rf.(plot_name);
if ~ishandle(fig_h), continue; end % Skip if figure closed

% Make sure figure is active
figure(fig_h);

ax_dist = fig_h.UserData.ax_dist;
ax_pwr = fig_h.UserData.ax_pwr;

% --- Set Limits based on ALL plotted data ---
min_range = Inf; max_range = -Inf;
min_power = Inf; max_power = -Inf;
min_sens = Inf;

% Get all line objects plotted on the distance axis
lines_dist = findobj(ax_dist, 'Type', 'line');
for i_line = 1:length(lines_dist)
ydata = get(lines_dist(i_line), 'YData');
ydata = ydata(isfinite(ydata)); % Ignore NaN/Inf
if ~isempty(ydata)
min_range = min(min_range, min(ydata));
max_range = max(max_range, max(ydata));
end
end

% Get all line objects plotted on the power axis
lines_pwr = findobj(ax_pwr, 'Type', 'line');
for i_line = 1:length(lines_pwr)
ydata = get(lines_pwr(i_line), 'YData');
ydata = ydata(isfinite(ydata)); % Ignore NaN/Inf
if ~isempty(ydata)
min_power = min(min_power, min(ydata));
max_power = max(max_power, max(ydata));
% Check if this is a sensitivity line (plotted with ':')
if strcmp(get(lines_pwr(i_line), 'LineStyle'), ':')
min_sens = min(min_sens, min(ydata)); % Track min sensitivity plotted
end
end
end

% Set X Limits
xlim(ax_dist, [time(1) time(end)]);

% Set Y Limits for Distance (Left Axis)
if isfinite(min_range) && isfinite(max_range)
range_r = max_range - min_range; if range_r < 1, range_r = 1; end % Avoid zero range
ylim(ax_dist, [min_range - 0.1*range_r, max_range + 0.1*range_r]);
else
ylim(ax_dist, [0 1]); % Default if no valid data
end
set(ax_dist, 'YColor', theme.labelColor); % Use theme color for axis

% Set Y Limits for Power (Right Axis) - Include sensitivity in range
combined_min = min([min_power, min_sens]); % Find overall min including sensitivity
combined_max = max_power; % Max is just max power

if isfinite(combined_min) && isfinite(combined_max)
range_p = combined_max - combined_min; if range_p < 1, range_p = 1; end % Avoid zero range
ylim(ax_pwr, [combined_min - 0.1*range_p, combined_max + 0.1*range_p]);
else
ylim(ax_pwr, [-120 -80]); % Default if no valid data
end
set(ax_pwr, 'YColor', theme.labelColor); % Use theme color for axis

% Add Legend for Dist/Pwr plot
if isfield(lgd_handles, plot_name)
lgd_info = lgd_handles.(plot_name);
% Filter out invalid handles before creating legend
valid_h_mask = ishandle(lgd_info.h);
if any(valid_h_mask)
lgd = legend(ax_dist, lgd_info.h(valid_h_mask), lgd_info.l(valid_h_mask), 'Location', 'best'); % Legend on left axis
set(lgd, 'TextColor', theme.textColor, 'Color', [theme.axesColor, 0.8], 'EdgeColor', theme.gridColor, 'FontSize', theme.fontSize-1); % Style it
end
end
hold(ax_dist, 'off'); % Release hold on primary axis
hold(ax_pwr, 'off'); % Release hold on secondary axis
drawnow; % Ensure updates are rendered
end
% --- End Finalizing Dist/Pwr Plots ---


             end % End Rx loop %
        end % End Tx loop %

        % --- Finalize All RF Plots (Add Legends, Release Hold, Save) ---
        all_field_names = fieldnames(figs_rf); % Get names of ALL fields in the struct
        % Filter out axes handle fields (those starting with 'ax')
        plot_names = all_field_names(~startsWith(all_field_names, 'ax')); % Keep only names NOT starting with 'ax'

        for i_plot = 1:length(plot_names) % Iterate through FIGURE names only
            plot_name = plot_names{i_plot}; % Now plot_name will be 'Status', 'RxPwr', 'DistPwr_GS1' etc.
            fig_h = figs_rf.(plot_name);    % This will now correctly get the FIGURE handle
            if ~ishandle(fig_h), continue; end % Skip if figure was closed %

            % Find the primary axes for adding legend (heuristics)
            ax_list = findobj(fig_h, 'Type', 'axes'); %
             ax_h = ax_list(1); % Default to first axes found %
             if length(ax_list) > 1 && strcmp(get(ax_list(1),'YAxisLocation'),'right'), ax_h=ax_list(2); end % Use left axes if yyaxis was used %

             if strcmp(plot_name, 'Status') && isfield(lgd_handles, 'Status') % Finalize Status plot Y-axis ticks/labels %
                  if ~isempty(lgd_handles.Status.ypos) %
                      set(ax_h, 'YTick', lgd_handles.Status.ypos, 'YTickLabel', lgd_handles.Status.labels, 'YLim', [0.5, link_counter + 0.5]); %
                      set(ax_h, 'YColor', theme.axesTickColor); % Make labels visible %
                  end %
                  hold(ax_h, 'off'); % Release hold %
             elseif isfield(lgd_handles, plot_name) % Add legends to other standard plots %
                   lgd_info = lgd_handles.(plot_name); %
                   if ~isempty(lgd_info.h) && all(ishandle(lgd_info.h)) % Ensure handles are valid %
                       lgd = legend(ax_h, lgd_info.h, lgd_info.l, 'Location', 'best'); % Create legend %
                       set(lgd, 'TextColor', theme.textColor, 'Color', [theme.axesColor, 0.8], 'EdgeColor', theme.gridColor, 'FontSize', theme.fontSize-1); % Style it %
                   end %
                   hold(ax_h, 'off'); % Release hold %
             end % (Combined Dist/Pwr plot legend handled implicitly if needed) %

             % Save the finalized plot using snapshot generator
             plot_filename_base = lower(strrep(plot_name,' ','_')); % e.g., 'rxpwr', 'distpwr_gs1' %
             % Use plot counter, not fig num, for snapshot index
             snapshot_generator(fig_h, plot_filename_base, config, i_plot+4); % Use relative plot index + offset %
        end %
        % next_fig_num = next_fig_num + length(plot_names); % Don't increment next_fig_num inside loop - Moved increment below

        % --- Plot 11: Link Availability Summary ---
        try %
            fig_avail = figure(next_fig_num); clf; set(fig_avail,'Name','Availability'); set_figure_style_helper_local(fig_avail,theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
            ax_avail = axes('Parent', fig_avail); hold(ax_avail,'on'); grid on; set_axes_style_helper_local(ax_avail,theme,is_cyberpunk); % CALLS EXTERNAL HELPER %
            % Calculate availability only for valid (non-skipped) links
            % Preallocate for performance (AGROW fix)
            max_links = 0; % Estimate max possible links %
            temp_tx_ids = fieldnames(processed_rf_data); %
            for temp_i = 1:length(temp_tx_ids) %
                 max_links = max_links + length(fieldnames(processed_rf_data.(temp_tx_ids{temp_i}))); %
            end %
            avail_labels = cell(1, max_links); %
            avail_pct = NaN(1, max_links); %
            avail_count = 0; %

            tx_ids_a = fieldnames(processed_rf_data); %
            for i_tx_a = 1:length(tx_ids_a) %
                rx_ids_a = fieldnames(processed_rf_data.(tx_ids_a{i_tx_a})); %
                for i_rx_a = 1:length(rx_ids_a) %
                    link_data_a = processed_rf_data.(tx_ids_a{i_tx_a}).(rx_ids_a{i_rx_a}); %
                    % Check if valid for availability calculation
                    if any(strcmp(link_data_a.status,'SKIP')) || ~isfield(link_data_a,'snr_db') || all(isnan(link_data_a.snr_db)), continue; end %

                    avail_count = avail_count + 1; %
                    avail_labels{avail_count} = sprintf('%s>%s',strrep(tx_ids_a{i_tx_a},'_',''),strrep(rx_ids_a{i_rx_a},'_','')); % Short label (AGROW fix) %
                    valid_mask_a = ~strcmp(link_data_a.status,'SKIP') & ~strcmp(link_data_a.status,'PENDING'); %
                    num_valid_steps_a = sum(valid_mask_a); %
                    if num_valid_steps_a > 0 %
                        ok_steps_a = sum(strcmp(link_data_a.status(valid_mask_a), 'OK')); %
                        avail_pct(avail_count) = (ok_steps_a / num_valid_steps_a) * 100; % AGROW fix %
                    else %
                        avail_pct(avail_count) = NaN; % Availability undefined if no valid steps %
                    end %
                end % End Rx loop for availability %
            end % End Tx loop for availability %

            % Trim preallocated arrays
            avail_labels = avail_labels(1:avail_count); %
            avail_pct = avail_pct(1:avail_count); %

            if avail_count > 0 %
                 barh(ax_avail, 1:avail_count, avail_pct, 'FaceColor', theme.barColor); % Horizontal bars %
                 set(ax_avail, 'YTick', 1:avail_count, 'YTickLabel', avail_labels); % Set labels on Y-axis %
                 xlabel('Availability (%)','Color',theme.labelColor); ylabel('Link','Color',theme.labelColor); %
                 title('Link Availability Summary','Color',theme.titleColor); xlim([0 105]); % X-axis 0-100% %
            else %
                 text(0.5, 0.5, 'No Valid RF Links for Availability Plot', 'Parent', ax_avail, 'HorizontalAlignment', 'center', 'Color', theme.textColor); %
                 ylim(ax_avail, [0 1]); % Set dummy limits if no bars plotted %
            end %
            hold(ax_avail, 'off'); %
            snapshot_generator(fig_avail, 'link_availability', config, next_fig_num); next_fig_num = next_fig_num + 1; % Increment fig num %
         catch ME_plot %
             % *** FIXED MEXCEP *** %
             warning(ME_plot.identifier, 'Failed to generate Plot 11 (Availability): %s', ME_plot.message); %
             if exist('fig_avail','var') && ishandle(fig_avail), try close(fig_avail); catch; end; end %
             next_fig_num = next_fig_num + 1; % Ensure counter increments %
         end % End Availability Plot try-catch %

    end % End if rf_data_available %

    fprintf('--- Post-Simulation Plot Generation Finished ---\n'); %

end % END OF FUNCTION visualization_model %


% =========================================================================
% LOCAL HELPER FUNCTIONS for visualization_model
% (These remain local as they are only used within this specific model)
% =========================================================================
% SEPEX warning handled by adding newline below

% --- Helper: Plot Terrain --- (Copied from Ray Trace) ---
function plot_terrain_helper_local(ax, terrain, lon_range, lat_range, theme) %
    % Plots terrain surface within specified Lat/Lon bounds on given axes.
    if ~terrain.data_loaded, return; end % Check if terrain data exists %
    dem_data = terrain.dem_data; R = terrain.ref_obj; %
    if isempty(dem_data) || isempty(R), return; end % Check data/ref obj not empty %

    grid_size = 300; % Number of grid points for terrain surface plot (adjust for detail vs speed) %
    lon_vec = linspace(lon_range(1), lon_range(2), grid_size); %
    lat_vec = linspace(lat_range(1), lat_range(2), grid_size); %
    [LON, LAT] = meshgrid(lon_vec, lat_vec); % Create grid of coordinates %
    ELEV_M = zeros(size(LON)); % Preallocate elevation grid %

    % Get elevation for each grid point (can be slow for large grids)
    for i = 1:grid_size %
        for j = 1:grid_size %
             try % Add try-catch around terrain lookup for robustness %
                  ELEV_M(i,j) = terrain.get_altitude(LAT(i,j), LON(i,j)); %
             catch ME_terr_lookup %
                  if i==1 && j==1 % Warn only once per plot call %
                      warning('visualization_model:TerrainLookupWarn', 'Terrain lookup failed during plotting: %s', ME_terr_lookup.message); %
                  end %
                  ELEV_M(i,j) = NaN; % Set failed lookups to NaN %
             end %
        end %
    end %

    ELEV_FT = ELEV_M * (1/0.3048); % Convert meters to feet for plotting %

    % Check if terrain is essentially flat within the plotted range
    if range(ELEV_FT(~isnan(ELEV_FT))) < 1 % Calculate range ignoring NaNs %
        % Plot as flat surface at mean altitude if variation is negligible
        mean_elev_ft = mean(ELEV_FT(~isnan(ELEV_FT))); if isnan(mean_elev_ft), mean_elev_ft = 0; end % Handle all NaN case %
        surf(ax, LON, LAT, ones(size(LON))*mean_elev_ft, 'FaceColor', theme.terrainFlatColor, 'EdgeColor', 'none', 'FaceAlpha', theme.terrainAlpha); %
    else %
        % Plot terrain using surf command
        surf(ax, LON, LAT, ELEV_FT, 'EdgeColor', 'none', 'FaceAlpha', theme.terrainAlpha); %
        colormap(ax, theme.terrainColormap); % Apply terrain colormap %
        material(ax,'dull'); % Set material properties for appearance %
    end %
end %

% --- Helper: Apply Theme Defaults --- (Copied from Ray Trace) ---
function theme_out = apply_theme_defaults_local(theme_in) %
    % Ensures essential theme fields exist, applying defaults if missing.
    % Prevents errors if theme file is incomplete or standard style is used.
    theme_out = theme_in; %
    % Define all fields potentially used in visualization_model and provide defaults
    defaults = struct(... %
        'bgColor',[1 1 1],'axesColor',[1 1 1],'axesTickColor',[0 0 0],'gridColor',[0.8 0.8 0.8],... %
        'textColor',[0 0 0],'textColorEmphasis',[1 0 0],'titleColor',[0 0 0],'labelColor',[0 0 0],... %
        'fontSize',10,'fontName','Helvetica','lineColors',{{[0 0.4470 0.7410]}},'linkColors',{{[0 0.4470 0.7410]}},... % Default MATLAB blue %
        'highlightColor',[1 0 0],'markerColorStart',[0 1 0],'markerColorEnd',[1 0 0],... %
        'markerColorWaypoint',[1 0 1],'markerColorGroundStation',[0 1 1],... %
        'statusOkColor',[0 1 0],'statusWarnColor',[1 0.5 0],'statusFailColor',[1 0 0],'statusUnknownColor',[0.5 0.5 0.5],... %
        'barColor',[0 0.4470 0.7410],'terrainColormap','terrain','terrainFlatColor',[0.8 0.8 0.8],'snrColormap','parula',... %
        'lineWidth',1.5,'markerSize',6,'terrainAlpha',0.6,'LINK_STATUS_PLOT_HEIGHT',0.15); %
    fields_def = fieldnames(defaults); %
    for i = 1:length(fields_def) %
        if ~isfield(theme_out, fields_def{i}) %
            theme_out.(fields_def{i}) = defaults.(fields_def{i}); %
        end %
    end %
end %