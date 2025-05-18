% =========================================================================
% TACTICAL REPLAY MODE (FINAL - Corrected v1.6) % <-- Version Update
% Version: FINAL 1.6
%
% Description:
%   Creates an interactive 3D playback interface for visualizing the completed
%   simulation results. Allows users to play, pause, step through time, adjust
%   playback speed, and view the aircraft's trajectory, orientation, and
%   RF link status relative to ground stations and terrain.
%
% Features:
%   - 3D visualization of flight path, waypoints, ground stations.
%   - Optional terrain surface display.
%   - Animated aircraft marker moving along the trajectory.
%   - Heading indicator for the aircraft marker.
%   - RF link lines connecting aircraft to ground stations, colored by status
%     (OK, Masked, Blocked, Skipped/Unknown) based on processed RF data.
%   - Timeline slider axis with clickable functionality to jump to specific times.
%   - UI Controls: Play/Pause, Step Forward/Backward buttons, Speed Slider.
%   - Real-time text display of key parameters (Time, Alt, Hdg).
%   - Improved axis scaling (`vis3d`) and interactive camera controls (Patch 9).
%   - Waypoint heading arrows displayed (Patch 8).
%
% Usage:
%   Called from `main_simulation.m` after results are processed.
%   `tactical_replay_mode(config, tspi_data, processed_rf_data, terrain, simulation_metadata);`
%   Blocks MATLAB execution until the replay figure window is closed (`uiwait`).
%
% Inputs:
%   config              - Main configuration struct.
%   tspi_data           - NxM matrix of TSPI data.
%   processed_rf_data   - Structured RF data (output of post_process_rf_log_helper).
%                         Used for coloring RF link lines based on status at each time step.
%   terrain             - Terrain model struct (output of `terrain_model.m`).
%                         Must contain the `get_altitude` function handle.
%   simulation_metadata - Metadata struct (contains `tspi_headers`).
%
% Dependencies:
%   - `cyberpunk_theme.m` (if cyberpunk style selected).
%   - `set_figure_style_helper_local.m` (external helper).
%   - `set_axes_style_helper_local.m` (external helper).
%   - Requires the helper function `plot_terrain_raytrace_helper_local` (defined at end).
%   - Requires `earth_model.m` for waypoint arrow calculation.
%
% Maintainer: [Your Name/Team]
% Last Updated: [2025-04-06] - Fixed terrain.get_altitude call (use function handle) (v1.6)
%                           - Fixed M2FT scope in terrain helper (v1.5)
%                           - Added isempty checks before length in RF update validation (v1.5)
%                           - Fixed RF link update logic in update_display_replay (v1.4)
%                           - Fixed ALL replay_state.idx -> replay_state.current_idx (v1.3)
% =========================================================================
function tactical_replay_mode(config, tspi_data, processed_rf_data, terrain, simulation_metadata) %
    % --- Input Checks & Basic Setup ---
    if ~isfield(config.visualization,'ENABLE_TACTICAL_REPLAY') || ~config.visualization.ENABLE_TACTICAL_REPLAY % Added isfield check %
        fprintf('[REPLAY] Tactical replay mode disabled in configuration.\n'); %
        return; %
    end %
    if isempty(tspi_data) %
         warning('[REPLAY] TSPI data is empty. Cannot start replay.'); %
         return; %
    end %
    rf_data_available = ~isempty(fieldnames(processed_rf_data)); % Check if the struct has any fields %
    if ~rf_data_available %
        fprintf('[REPLAY] Processed RF data is empty. RF link lines will be default gray.\n'); %
    end %
    % --- V1.6 Check: Validate Terrain Struct ---
    if ~isstruct(terrain) || ~isfield(terrain,'get_altitude') || ~isa(terrain.get_altitude, 'function_handle')
         warning('TacticalReplay:InvalidTerrainInput','Input terrain argument is not a valid struct with get_altitude function handle. Terrain plotting disabled.');
         terrain.data_loaded = false; % Prevent terrain plotting attempt
    end
    % --- End V1.6 Check ---
    fprintf('[REPLAY] Initializing Tactical Replay Mode...\n'); %

    % --- Theme ---
    theme = struct(); is_cyberpunk = false; %
    if isfield(config.visualization,'PLOT_STYLE') && strcmpi(config.visualization.PLOT_STYLE, 'cyberpunk') % Added isfield check %
        try theme = cyberpunk_theme(); is_cyberpunk = true; %
        catch ME_theme, warning('[REPLAY] Failed to load cyberpunk theme: %s.', ME_theme.message); end %
    end %
    theme = apply_theme_defaults_replay_local(theme); % Apply defaults for robustness %

    % --- Extract Data ---
    time      = tspi_data(:, 1); %
    num_steps = length(time); %
    if num_steps < 2, warning('[REPLAY] Need at least 2 TSPI steps for replay.'); return; end %
    dt = mean(diff(time)); % Use mean difference for potentially variable time steps %

    % Use headers for robust column extraction
    hdrs      = lower(simulation_metadata.tspi_headers); % Ensure lowercase for strcmp %
    lat_col   = find(strcmp(hdrs, 'lat_deg'), 1); %
    lon_col   = find(strcmp(hdrs, 'lon_deg'), 1); %
    alt_col   = find(strcmp(hdrs, 'alt_m_msl'), 1); %
    hdg_col   = find(strcmp(hdrs, 'heading_deg'), 1); %

    % Check if essential columns were found
    if isempty(lat_col) || isempty(lon_col) || isempty(alt_col) || isempty(hdg_col) %
        error('[REPLAY] Could not find required Lat/Lon/Alt/Heading columns in TSPI data. Check tspi_headers.'); % More specific error %
    end %

    lat       = tspi_data(:, lat_col); %
    lon       = tspi_data(:, lon_col); %
    alt_m     = tspi_data(:, alt_col); %
    alt_ft    = alt_m * (1/0.3048); % Convert altitude to feet for display %
    heading   = tspi_data(:, hdg_col); %

    % --- Constants ---
    M2FT = 1 / 0.3048; % Define M2FT in main scope %
    DEG2RAD = pi/180; %

    % --- Create Figure and Axes ---
    fig_pos = config.visualization.FIGURE_POSITION; % Get position from config %
    fig = figure('Name', 'Tactical Replay', 'Position', fig_pos); % Use variable %
    set_figure_style_helper_local(fig, theme, is_cyberpunk); % Apply theme (CALLS EXTERNAL HELPER) %

    % Main 3D Axes (takes up most of the figure)
    ax_main = axes('Parent', fig, 'Position', [0.05, 0.15, 0.9, 0.8]); % Leave space at bottom for controls %
    hold(ax_main, 'on'); grid(ax_main, 'on'); view(ax_main, 3); % Enable 3D view %
    set_axes_style_helper_local(ax_main, theme, is_cyberpunk); % Apply theme (CALLS EXTERNAL HELPER) %
    title(ax_main, 'Tactical Replay', 'Color', theme.titleColor, 'FontSize', theme.fontSize + 1, 'FontName', theme.fontName); %
    xlabel(ax_main, 'Longitude (deg)', 'Color', theme.labelColor); %
    ylabel(ax_main, 'Latitude (deg)', 'Color', theme.labelColor); %
    zlabel(ax_main, 'Altitude (ft MSL)', 'Color', theme.labelColor); %

    % Timeline Axes (at the bottom)
    ax_timeline = axes('Parent', fig, 'Position', [0.05, 0.05, 0.7, 0.05]); % Position left of controls %
    hold(ax_timeline, 'on'); %

    % Apply theme, hide Y ticks/label
    set_axes_style_helper_local(ax_timeline, theme, is_cyberpunk); % (CALLS EXTERNAL HELPER) %
    set(ax_timeline, 'YColor', 'none', 'YTick', []); % Hide Y elements %
    set(ax_timeline, 'FontSize', theme.fontSize - 1); %
    xlabel(ax_timeline, 'Time (s)', 'Color', theme.labelColor); %

    % =====================================================================
    % --- Plot Static Elements (Background) ---
    % =====================================================================

    % --- Full Trajectory Path (Dimmed) ---
    plot3(ax_main, lon, lat, alt_ft, '-', 'Color', [theme.lineColors{1}, 0.3], 'LineWidth', 1); % Use transparency %

    % --- Waypoints ---
    wp = config.flight.waypoints; %
    if ~isempty(wp) % Check if waypoints exist %
        plot3(ax_main, wp(:,2), wp(:,1), wp(:,3)*M2FT, '^', ... % Plot Alt in feet %
              'MarkerSize', theme.markerSize, 'Color', theme.markerColorWaypoint, 'MarkerFaceColor', theme.markerColorWaypoint); %
        % Add waypoint labels
        for i = 1:size(wp,1) %
            text(ax_main, wp(i,2), wp(i,1), wp(i,3)*M2FT, sprintf(' WP%d', i), ... %
                 'Color', theme.textColor, 'FontSize', theme.fontSize-1, 'FontName', theme.fontName, ... %
                 'VerticalAlignment', 'bottom'); %
        end %
    end %

    % --- Waypoint Heading Arrows (Patch 8) ---
    if ~isempty(wp) && size(wp, 2) >= 5 && isfield(config.visualization,'WAYPOINT_ARROW_SCALE') % Check if TargetHeading column exists & scale param exists %
        arrow_scale_m = config.visualization.WAYPOINT_ARROW_SCALE; % Length in meters from config %
        earth_model_temp = earth_model(config.earth.model); % Need earth model for 'move' %
        for i = 1:size(wp, 1) %
            wp_lat = wp(i,1); wp_lon = wp(i,2); wp_alt_ft = wp(i,3)*M2FT; %
            wp_hdg = wp(i,5); % Get commanded heading at WP %
            if ~isnan(wp_hdg) % Only plot if heading command exists %
                % Calculate endpoint of arrow in Lat/Lon using earth model
                [arrow_lat, arrow_lon] = earth_model_temp.move(wp_lat, wp_lon, arrow_scale_m, wp_hdg); %
                % Use quiver3 to draw arrow: Start(x,y,z), Delta(dx,dy,dz), Scale(0=off)
                quiver3(ax_main, wp_lon, wp_lat, wp_alt_ft, arrow_lon-wp_lon, arrow_lat-wp_lat, 0, 0, ... %
                        'Color', theme.markerColorWaypoint, 'LineWidth', 1, 'MaxHeadSize', 0.5); %
            end %
        end %
    end %

    % --- Ground Stations ---
    gs_plots = []; gs_labels = {}; % For legend %
    num_gs = 0; % Count ground stations %
    if isfield(config.rf, 'ground_stations') && ~isempty(config.rf.ground_stations) && iscell(config.rf.ground_stations) % Add iscell check %
        num_gs = length(config.rf.ground_stations); %
        for i_gs = 1:num_gs %
            gs_cfg    = config.rf.ground_stations{i_gs}; %
            % --- Robustness: Check if essential fields exist ---
            if ~isfield(gs_cfg,'lon_deg') || ~isfield(gs_cfg,'lat_deg') || ~isfield(gs_cfg,'alt_m_msl') || ~isfield(gs_cfg,'id') %
                warning('TacticalReplay:GroundStationConfigWarn', 'Ground station %d is missing required fields (lon_deg, lat_deg, alt_m_msl, id). Skipping.', i_gs); % Added ID %
                continue; % Skip this ground station %
            end %
            % --- End Robustness Check ---
            gs_alt_ft = gs_cfg.alt_m_msl * M2FT; % Use constant from main scope %
            h_gs      = plot3(ax_main, gs_cfg.lon_deg, gs_cfg.lat_deg, gs_alt_ft, 's', ... %
                              'MarkerSize', theme.markerSize+2, 'Color', theme.markerColorGroundStation, ... %
                              'MarkerFaceColor', theme.markerColorGroundStation); %
            text(ax_main, gs_cfg.lon_deg, gs_cfg.lat_deg, gs_alt_ft, sprintf(' %s', gs_cfg.id), ... %
                 'Color', theme.textColor, 'FontSize', theme.fontSize-1, 'FontName', theme.fontName, ... %
                 'VerticalAlignment', 'bottom'); %
            % Store first handle for legend
            if isempty(gs_plots), gs_plots = h_gs; gs_labels = {'Ground Station'}; end % Assign first valid plot %
        end %
    end %

    % --- Plot Terrain Surface (Optional) ---
    if isfield(terrain,'data_loaded') && terrain.data_loaded && ... % Check terrain struct validity FIRST %
       isfield(config.visualization, 'ENABLE_TERRAIN_VIEW') && config.visualization.ENABLE_TERRAIN_VIEW % Added isfield check %
        try %
            fprintf('[REPLAY] Plotting terrain surface...\n'); %
            % Use the helper function defined locally at the end of this file
            % PASS M2FT and the terrain struct to the helper function
            plot_terrain_raytrace_helper_local(ax_main, terrain, xlim(ax_main), ylim(ax_main), theme, M2FT); % PASS terrain struct %
        catch ME_terr_plot %
            warning('[REPLAY] Failed to plot terrain: %s', ME_terr_plot.message); %
        end %
    end %

    % =====================================================================
    % --- Dynamic Replay Elements (Handles stored for updates) ---
    % =====================================================================
    % (Aircraft Marker, Heading Indicator, RF Link Lines, Info Text, Time Marker remain the same as v1.5)
    % ... [Code identical to v1.5] ...
    % --- Aircraft Marker ---
    h_aircraft = plot3(ax_main, lon(1), lat(1), alt_ft(1), 'p', ... % Pentagon marker %
                       'MarkerSize', theme.markerSize + 4, 'Color', theme.highlightColor, ... %
                       'MarkerFaceColor', theme.highlightColor); %
    aircraft_label = {'Aircraft'}; % Label for legend %

    % --- Heading Indicator ---
    heading_len_factor_lon = diff(xlim(ax_main)) * 0.05; % Scale based on lon range %
    heading_len_factor_lat = diff(ylim(ax_main)) * 0.05; % Scale based on lat range %
    head_lon_end = lon(1) + heading_len_factor_lon * sind(heading(1)); % East component %
    head_lat_end = lat(1) + heading_len_factor_lat * cosd(heading(1)); % North component %
    h_heading = line(ax_main, [lon(1), head_lon_end], [lat(1), head_lat_end], [alt_ft(1), alt_ft(1)], ... %
                     'Color', theme.highlightColor, 'LineWidth', 2); %

    % --- RF Link Lines ---
    h_rf_links = gobjects(num_gs, 1); % One line handle per ground station configured %
    if num_gs > 0 %
        valid_gs_count = 0; % Count valid lines created %
        for i_gs = 1:length(config.rf.ground_stations) % Iterate original config length
            gs_cfg = config.rf.ground_stations{i_gs}; %
             if ~isfield(gs_cfg,'lon_deg') || ~isfield(gs_cfg,'lat_deg') || ~isfield(gs_cfg,'alt_m_msl') || ~isfield(gs_cfg,'id') % Check again %
                 continue; % Skip if essential fields missing %
             end %
             valid_gs_count = valid_gs_count + 1; % Increment valid count
             gs_alt_ft = gs_cfg.alt_m_msl * M2FT; % Use constant from main scope %
             h_rf_links(valid_gs_count) = line(ax_main, [lon(1), gs_cfg.lon_deg], [lat(1), gs_cfg.lat_deg], [alt_ft(1), gs_alt_ft], ... %
                 'Color', [theme.statusUnknownColor, 0.5], 'LineWidth', 1, 'LineStyle', ':'); % Use Unknown color initially %
             % Store GS ID in UserData for easier lookup during updates
             set(h_rf_links(valid_gs_count), 'UserData', gs_cfg.id);
        end %
        h_rf_links = h_rf_links(1:valid_gs_count); % Trim unused handles %
        num_gs = valid_gs_count; % Update num_gs to reflect only valid lines created %
    end %

    % --- Information Text Panel ---
    info_str_format = 'T: %.1fs\nAlt: %.0fft\nHdg: %.1f°'; % Format string %
    h_info = text(ax_main, 0, 0, 0, sprintf(info_str_format, time(1), alt_ft(1), heading(1)), ... % Initial text %
                  'Color', theme.textColor, 'FontSize', theme.fontSize - 1, 'FontName', theme.fontName, ... %
                  'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', ... %
                  'BackgroundColor', [theme.axesColor, 0.7], 'EdgeColor', theme.gridColor); % Semi-transparent background %

    % --- Timeline Axis Content ---
    plot(ax_timeline, time, zeros(size(time)), '-', 'Color', theme.textColor, 'LineWidth', 1); % Line representing time axis %
    h_time_marker = plot(ax_timeline, time(1), 0, 'o', 'MarkerSize', theme.markerSize, ... %
                         'Color', theme.highlightColor, 'MarkerFaceColor', theme.highlightColor); %
    xlim(ax_timeline, [time(1), time(end)]); %
    ylim(ax_timeline, [-1, 1]); % Hide Y extent %


    % =====================================================================
    % --- Axis Scaling and Viewport (Patch 9) ---
    % =====================================================================
    % (Axis scaling code remains the same as v1.5)
    % ... [Code identical to v1.5] ...
    gs_lons = []; gs_lats = []; gs_alts = []; % Initialize %
    if isfield(config.rf, 'ground_stations') && ~isempty(config.rf.ground_stations) && iscell(config.rf.ground_stations) % Check if not empty and is cell %
         try gs_lons_cell = cellfun(@(x) ifelse(isfield(x,'lon_deg'), x.lon_deg, NaN), config.rf.ground_stations, 'UniformOutput', false); gs_lons = cell2mat(gs_lons_cell); catch; gs_lons = []; end % Added ifelse %
         try gs_lats_cell = cellfun(@(x) ifelse(isfield(x,'lat_deg'), x.lat_deg, NaN), config.rf.ground_stations, 'UniformOutput', false); gs_lats = cell2mat(gs_lats_cell); catch; gs_lats = []; end % Added ifelse %
         try gs_alts_cell = cellfun(@(x) ifelse(isfield(x,'alt_m_msl'), x.alt_m_msl*M2FT, NaN), config.rf.ground_stations, 'UniformOutput', false); gs_alts = cell2mat(gs_alts_cell); catch; gs_alts = []; end % Added ifelse % Use M2FT %
    end %
    gs_lons = gs_lons(~isnan(gs_lons)); gs_lats = gs_lats(~isnan(gs_lats)); gs_alts = gs_alts(~isnan(gs_alts)); % Filter NaNs %
    all_lons = [lon(:); wp(:,2); gs_lons(:)]; %
    all_lats = [lat(:); wp(:,1); gs_lats(:)]; %
    all_alts_ft = [alt_ft(:); wp(:,3)*M2FT; gs_alts(:)]; % Use M2FT %
    terrain_surf_rep = findobj(ax_main, 'Type', 'surface'); %
    if ~isempty(terrain_surf_rep) %
        z_data_terr = terrain_surf_rep.ZData; % Get ZData %
        z_data_terr = z_data_terr(~isnan(z_data_terr)); % Remove NaNs %
        if ~isempty(z_data_terr) % Check if any valid Z data exists %
             all_alts_ft = [all_alts_ft; min(z_data_terr); max(z_data_terr)]; %
        end %
    end %
    min_lon = min(all_lons); max_lon = max(all_lons); range_lon = max(max_lon - min_lon, 0.01); % Min 0.01 deg range %
    min_lat = min(all_lats); max_lat = max(all_lats); range_lat = max(max_lat - min_lat, 0.01); %
    min_alt = min(all_alts_ft); max_alt = max(all_alts_ft); range_alt = max(max_alt - min_alt, 100); % Min 100 ft range %
    pad_lon = range_lon * 0.1; pad_lat = range_lat * 0.1; pad_alt = range_alt * 0.1 + 100; % Min 100ft alt padding %
    xlim(ax_main, [min_lon - pad_lon, max_lon + pad_lon]); %
    ylim(ax_main, [min_lat - pad_lat, max_lat + pad_lat]); %
    zlim(ax_main, [min_alt - pad_alt, max_alt + pad_alt]); %
    axis(ax_main, 'vis3d'); %
    view(ax_main, -35, 30); % Azimuth, Elevation %

    % =====================================================================
    % --- UI Controls (Positioned at bottom of figure) ---
    % =====================================================================
    % (UI Controls code remains the same as v1.5)
    % ... [Code identical to v1.5] ...
    ui_ypos   = 0.01; ui_height = 0.04; ui_gap = 0.01; %
    ui_width_btn_step = 0.03; ui_width_btn_play = 0.06; ui_width_slider = 0.1; ui_width_lbl = 0.05; %
    start_x   = 0.75; %
    pos = [start_x, ui_ypos, ui_width_btn_step, ui_height]; %
    btn_step_back = uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '<', 'Units','normalized', 'Position', pos, 'FontName', theme.fontName); %
    if is_cyberpunk, set(btn_step_back, 'BackgroundColor', theme.axesColor, 'ForegroundColor', theme.highlightColor); end %
    pos = [pos(1)+pos(3)+ui_gap, ui_ypos, ui_width_btn_play, ui_height]; %
    btn_play = uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', 'Play', 'Units','normalized', 'Position', pos, 'FontName', theme.fontName); %
    if is_cyberpunk, set(btn_play, 'BackgroundColor', theme.axesColor, 'ForegroundColor', theme.highlightColor); end %
    pos = [pos(1)+pos(3)+ui_gap, ui_ypos, ui_width_btn_step, ui_height]; %
    btn_step_fwd = uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '>', 'Units','normalized', 'Position', pos, 'FontName', theme.fontName); %
    if is_cyberpunk, set(btn_step_fwd, 'BackgroundColor', theme.axesColor, 'ForegroundColor', theme.highlightColor); end %
    pos = [pos(1)+pos(3)+ui_gap, ui_ypos, ui_width_lbl, ui_height]; %
    lbl_speed = uicontrol('Parent', fig, 'Style', 'text', 'String', 'Speed:', 'Units','normalized', 'Position', pos, 'FontName', theme.fontName, 'HorizontalAlignment','right'); %
    if is_cyberpunk, set(lbl_speed, 'BackgroundColor', theme.bgColor, 'ForegroundColor', theme.textColor); end %
    pos = [pos(1)+pos(3), ui_ypos, ui_width_slider, ui_height]; %
    speed_slider = uicontrol('Parent', fig, 'Style', 'slider', 'Min', 0.1, 'Max', 10, 'Value', 1, 'Units','normalized', 'Position', pos); %
    if is_cyberpunk, set(speed_slider, 'BackgroundColor', theme.axesColor); end %

    % =====================================================================
    % --- Replay State & Callback Setup ---
    % =====================================================================
    % (Replay State & Callback Setup code remains the same as v1.5)
    % ... [Code identical to v1.5] ...
    replay_state = struct(); %
    replay_state.is_playing = false; replay_state.current_idx = 1; %
    replay_state.speed      = get(speed_slider, 'Value'); replay_state.max_idx = num_steps; %
    replay_state.timer      = []; timer_period = dt; %
    set(btn_play,      'Callback', @cb_play); set(btn_step_fwd,  'Callback', @cb_step_fwd); %
    set(btn_step_back, 'Callback', @cb_step_back); set(speed_slider,  'Callback', @cb_speed); %
    set(ax_timeline,   'ButtonDownFcn', @cb_timeline); set(fig, 'CloseRequestFcn', @cb_close); %
    try cameratoolbar(fig, 'Show'); catch; end; try rotate3d(ax_main, 'on'); catch; end %
    update_display_replay(1); update_info_position_replay(); %
    fprintf('[REPLAY] Tactical replay mode initialized. Use UI controls or close window to exit.\n'); %
    try uiwait(fig);
    catch ME_uiwait; if isvalid(fig) && ~strcmp(ME_uiwait.identifier,'MATLAB:Figure:BeingDeleted'); rethrow(ME_uiwait); end; end %

% =========================================================================
%                       NESTED CALLBACK FUNCTIONS
% =========================================================================
% (Callbacks: update_display_replay, update_info_position_replay, cb_timer,
%  cb_play, cb_step_fwd, cb_step_back, cb_speed, cb_timeline, cb_close
%  remain the same as v1.5 EXCEPT for RF link update part in update_display_replay)

    % --- Update Display Function ---
% --- Update Display Function ---
function update_display_replay(idx) %
     % ... [Initial checks and updates for AC marker, heading, time marker, info text identical to v1.5] ...
     if ~isnumeric(idx) || idx < 1 || idx > replay_state.max_idx, warning('TacticalReplay:InvalidIndex','Invalid index (%d) passed to update_display_replay.', idx); return; end %
     set(h_aircraft, 'XData', lon(idx), 'YData', lat(idx), 'ZData', alt_ft(idx)); %
     head_lon_end = lon(idx) + heading_len_factor_lon * sind(heading(idx)); %
     head_lat_end = lat(idx) + heading_len_factor_lat * cosd(heading(idx)); %
     set(h_heading, 'XData', [lon(idx), head_lon_end], 'YData', [lat(idx), head_lat_end], 'ZData', [alt_ft(idx), alt_ft(idx)]); %
     set(h_time_marker, 'XData', time(idx)); %
     set(h_info, 'String', sprintf(info_str_format, time(idx), alt_ft(idx), heading(idx))); %
     update_info_position_replay(); %

     % ============================================================== %
     % --- V1.6: RF LINK UPDATE LOGIC (Using UserData for Robustness) ---
     % ============================================================== %
     if rf_data_available && num_gs > 0 && all(isgraphics(h_rf_links)) % Check handles are valid %
          try % Wrap the entire RF update logic for safety %
              % Iterate through each VALID RF link line handle created
              for i_link = 1:num_gs % Use num_gs which reflects valid handles %
                  link_handle = h_rf_links(i_link); % Get the handle %
                  target_rx_id_rep = get(link_handle, 'UserData'); % Get GS ID stored in UserData %

                  % Find corresponding GS config (needed for position)
                  gs_match_idx = find(strcmp({config.rf.ground_stations{:}.id}, target_rx_id_rep), 1);
                  if isempty(gs_match_idx), set(link_handle, 'Visible', 'off'); continue; end % Hide if no match %
                  gs_cfg_rep = config.rf.ground_stations{gs_match_idx};
                  rx_lon_rep = gs_cfg_rep.lon_deg; % Longitude of this GS %
                  rx_lat_rep = gs_cfg_rep.lat_deg; % Latitude of this GS %
                  rx_alt_ft_rep = gs_cfg_rep.alt_m_msl * M2FT; % Altitude (ft) of this GS %

                  % --- Search processed_rf_data for a link TO this receiver at the current time step ---
                  found_link_data_rep = []; % Initialize flag/data holder %
                  tx_ids_rep = fieldnames(processed_rf_data); % Get all transmitter IDs from the data %
                  for i_tx_rep = 1:length(tx_ids_rep) % Loop through transmitters found in data %
                      current_tx_id_rep = tx_ids_rep{i_tx_rep}; % Get the current Tx ID string %
                      if isfield(processed_rf_data.(current_tx_id_rep), target_rx_id_rep) % Check if Rx field exists for this Tx %
                          link_data_candidate = processed_rf_data.(current_tx_id_rep).(target_rx_id_rep); % Get the data struct %

                          % --- V1.6 DEBUG LOGGING (Moved Earlier - KEEP THIS ACTIVE) --- START ---
                          fprintf('DEBUG REPLAY (Candidate): Idx=%d, RxID=%s, TxID=%s\n', idx, target_rx_id_rep, current_tx_id_rep);
                          fprintf('  > link_data_candidate class: %s\n', class(link_data_candidate));
                          if isstruct(link_data_candidate) && isfield(link_data_candidate, 'status')
                              fprintf('  > .status class: %s, size: [%s]\n', class(link_data_candidate.status), mat2str(size(link_data_candidate.status)));
                               % Check if idx is valid before attempting access for logging
                               if iscell(link_data_candidate.status) && idx <= length(link_data_candidate.status) && idx > 0
                                  try
                                      status_val_str = 'Error accessing element'; status_element = link_data_candidate.status{idx};
                                      if ischar(status_element), status_val_str = status_element;
                                      elseif iscell(status_element), status_val_str = sprintf('Nested Cell! Contents: "%s"', strjoin(status_element,'" "'));
                                      else, status_val_str = mat2str(status_element); end
                                      fprintf('  > .status{%d} potential value: %s\n', idx, status_val_str);
                                  catch ME_log_access, fprintf('  > .status{%d} access check FAILED: %s\n', idx, ME_log_access.message); end
                               else, fprintf('  > .status{%d} - Index invalid or status not cell.\n', idx); end
                          else, fprintf('  > .status field MISSING or candidate not struct\n'); end
                          fprintf('  -----------------------------------\n');
                          % --- V1.6 DEBUG LOGGING (Moved Earlier) --- END ---

                          % --- V1.5/1.6 Validation Logic (remains here) ---
                          is_valid_struct = isstruct(link_data_candidate);
                          has_status = is_valid_struct && isfield(link_data_candidate,'status') && iscell(link_data_candidate.status) && ~isempty(link_data_candidate.status);
                          has_terrain = is_valid_struct && isfield(link_data_candidate,'terrain_blocked') && islogical(link_data_candidate.terrain_blocked) && ~isempty(link_data_candidate.terrain_blocked);
                          has_mask = is_valid_struct && isfield(link_data_candidate,'aircraft_masked') && islogical(link_data_candidate.aircraft_masked) && ~isempty(link_data_candidate.aircraft_masked);
                          idx_ok_status = has_status && idx <= length(link_data_candidate.status);
                          idx_ok_terrain = has_terrain && idx <= length(link_data_candidate.terrain_blocked);
                          idx_ok_mask = has_mask && idx <= length(link_data_candidate.aircraft_masked);
                          if is_valid_struct && has_status && has_terrain && has_mask && idx_ok_status && idx_ok_terrain && idx_ok_mask % Check ALL conditions %
                              found_link_data_rep = link_data_candidate; % Store the valid data struct %
                              break; % Stop searching Tx's for this GS, use the first one found %
                          end % --- End Validation ---
                      end % End if receiver field exists %
                  end % End loop through transmitters %

                  % --- Update the line style/color based on FOUND link data (or default) ---
                  if ~isempty(found_link_data_rep) % If we found valid data for this GS receiver %
                      % --- V1.7: Robust Access --- START ---
                      current_status_rep = 'PENDING'; % Default status
                      is_terrain_blocked_rep = true;   % Default to blocked/masked for safety
                      is_aircraft_masked_rep = true;

                      try % Add specific try-catch around access
                          % Retrieve fields first
                          status_data = found_link_data_rep.status;
                          terrain_data = found_link_data_rep.terrain_blocked;
                          mask_data = found_link_data_rep.aircraft_masked;

                          % Check status field and access
                          if iscell(status_data) && idx > 0 && idx <= numel(status_data)
                              status_element = status_data{idx};
                              if ischar(status_element) % Expecting a char array (string)
                                  current_status_rep = status_element;
                              elseif iscell(status_element) && ~isempty(status_element) && ischar(status_element{1})
                                  % Handle nested cell containing a string (potential issue source)
                                  current_status_rep = status_element{1};
                                  persistent nested_cell_warn; if isempty(nested_cell_warn), warning('TacticalReplay:NestedStatusCell','Found nested cell in status field at Idx=%d, Rx=%s. Using first element.',idx, target_rx_id_rep); nested_cell_warn=true; end
                              else
                                   if isempty(nested_cell_warn), warning('TacticalReplay:InvalidStatusType','Unexpected status type (%s) at Idx=%d, Rx=%s.', class(status_element), idx, target_rx_id_rep); nested_cell_warn=true; end
                              end
                          else
                              if isempty(nested_cell_warn), warning('TacticalReplay:InvalidStatusStruct','Status field invalid (class: %s, size: [%s]) or index %d out of bounds for Rx=%s.', class(status_data), mat2str(size(status_data)), idx, target_rx_id_rep); nested_cell_warn=true; end
                          end

                          % Check terrain field and access
                          if islogical(terrain_data) && idx > 0 && idx <= numel(terrain_data)
                              is_terrain_blocked_rep = terrain_data(idx);
                          else
                               if isempty(nested_cell_warn), warning('TacticalReplay:InvalidTerrainStruct','Terrain field invalid or index %d out of bounds for Rx=%s.', idx, target_rx_id_rep); nested_cell_warn=true; end
                          end

                          % Check mask field and access
                          if islogical(mask_data) && idx > 0 && idx <= numel(mask_data)
                              is_aircraft_masked_rep = mask_data(idx);
                          else
                               if isempty(nested_cell_warn), warning('TacticalReplay:InvalidMaskStruct','Mask field invalid or index %d out of bounds for Rx=%s.', idx, target_rx_id_rep); nested_cell_warn=true; end
                          end
                      catch ME_access
                           if isempty(nested_cell_warn), warning('TacticalReplay:DataAccessError','Error accessing RF data fields at index %d for Rx=%s: %s. Using defaults.', idx, target_rx_id_rep, ME_access.message); nested_cell_warn=true; end
                           % Defaults assigned above will be used
                      end
                      % --- V1.7: Robust Access --- END ---

                      % Determine color based on retrieved/defaulted values
                      ray_color_rep = theme.statusUnknownColor; % Default %
                      if strcmp(current_status_rep, 'SKIP'), ray_color_rep = theme.statusUnknownColor;
                      elseif is_terrain_blocked_rep, ray_color_rep = theme.statusFailColor;
                      elseif is_aircraft_masked_rep, ray_color_rep = theme.statusWarnColor;
                      elseif strcmp(current_status_rep, 'OK'), ray_color_rep = theme.statusOkColor;
                      elseif strcmp(current_status_rep, 'FAIL'), ray_color_rep = theme.statusFailColor;
                      elseif strcmp(current_status_rep, 'PENDING'), ray_color_rep = theme.statusUnknownColor; % Handle pending state
                      end %
                      set(link_handle, ... % Use the correct handle from the outer loop %
                          'XData', [lon(idx), rx_lon_rep], 'YData', [lat(idx), rx_lat_rep], 'ZData', [alt_ft(idx), rx_alt_ft_rep], ...
                          'Color', [ray_color_rep, 0.7], 'LineStyle', '-', 'Visible', 'on'); % Ensure visible %
                  else % No valid link data found for this receiver at this time step %
                       set(link_handle, ...
                          'XData', [lon(idx), rx_lon_rep], 'YData', [lat(idx), rx_lat_rep], 'ZData', [alt_ft(idx), rx_alt_ft_rep], ...
                          'Color', [theme.statusUnknownColor, 0.5], 'LineStyle', ':', 'Visible', 'on'); % Ensure visible %
                  end % End if data found for this GS %
              end % End loop through link handles (i_link) %
          catch ME_rf_update % Catch errors during the RF update process %
               persistent rf_update_warning_shown; if isempty(rf_update_warning_shown), warning('TacticalReplay:RFUpdateError','Error updating RF link visuals (further warnings suppressed): %s', ME_rf_update.message); rf_update_warning_shown = true; end % Added ID %
               % Display stack trace for debugging the catch block itself if needed
               % ME_rf_update.getReport
          end % End try-catch for RF update %
      end % end if rf_data_available and graphics handles valid %
      % ============================================================== %
      % --- END V1.6 RF LINK UPDATE CORRECTION ---
      % ============================================================== %
      drawnow; %
 end % End update_display_replay %


     % --- Helper to reposition Info Text ---
     function update_info_position_replay() %
         try lim_x = xlim(ax_main); lim_y = ylim(ax_main); lim_z = zlim(ax_main); %
             set(h_info, 'Position', [lim_x(1) + 0.02*diff(lim_x), lim_y(2) - 0.02*diff(lim_y), lim_z(2)]); %
         catch ME_setpos, warning('TacticalReplay:SetPositionWarn', 'Could not update info text position: %s', ME_setpos.message); end %
     end %
     % --- Timer Callback ---
     function cb_timer(~, ~), if replay_state.is_playing, if replay_state.current_idx < replay_state.max_idx, replay_state.current_idx = replay_state.current_idx + 1; update_display_replay(replay_state.current_idx); else; cb_play(); end; end; end %
     % --- Play/Pause Button Callback ---
     function cb_play(~, ~), if ~replay_state.is_playing && replay_state.current_idx >= replay_state.max_idx, replay_state.current_idx = 1; update_display_replay(1); end; replay_state.is_playing = ~replay_state.is_playing; if replay_state.is_playing, set(btn_play, 'String', 'Pause'); if isempty(replay_state.timer) || ~isvalid(replay_state.timer), replay_state.timer = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', max(0.01, timer_period / replay_state.speed), 'TimerFcn', @cb_timer); else; stop(replay_state.timer); set(replay_state.timer, 'Period', max(0.01, timer_period / replay_state.speed)); end; start(replay_state.timer); else; set(btn_play, 'String', 'Play'); if ~isempty(replay_state.timer) && isvalid(replay_state.timer) && strcmp(replay_state.timer.Running, 'on'), stop(replay_state.timer); end; end; end %
     % --- Step Forward Button Callback ---
     function cb_step_fwd(~, ~), if replay_state.is_playing, cb_play(); end; if replay_state.current_idx < replay_state.max_idx, replay_state.current_idx = replay_state.current_idx + 1; update_display_replay(replay_state.current_idx); end; end %
      % --- Step Backward Button Callback ---
      function cb_step_back(~, ~), if replay_state.is_playing, cb_play(); end; if replay_state.current_idx > 1, replay_state.current_idx = replay_state.current_idx - 1; update_display_replay(replay_state.current_idx); end; end %
     % --- Speed Slider Callback ---
     function cb_speed(~, ~), replay_state.speed = get(speed_slider, 'Value'); if ~isempty(replay_state.timer) && isvalid(replay_state.timer), was_playing = replay_state.is_playing; if was_playing, stop(replay_state.timer); end; set(replay_state.timer, 'Period', max(0.01, timer_period / replay_state.speed)); if was_playing, start(replay_state.timer); end; end; end %
     % --- Timeline Click Callback ---
      function cb_timeline(~, event), if replay_state.is_playing, cb_play(); end; if event.Button == 1, click_time = event.IntersectionPoint(1); [~, temp_idx] = min(abs(time - click_time)); replay_state.current_idx = max(1, min(temp_idx, replay_state.max_idx)); update_display_replay(replay_state.current_idx); end; end %
      % --- Figure Close Request Callback ---
      function cb_close(~, ~), fprintf('[REPLAY] Close requested.\n'); if ~isempty(replay_state.timer) && isvalid(replay_state.timer), if strcmp(replay_state.timer.Running, 'on'), stop(replay_state.timer); fprintf('[REPLAY] Timer stopped.\n'); end; delete(replay_state.timer); replay_state.timer = []; fprintf('[REPLAY] Timer deleted.\n'); end; try delete(fig); fprintf('[REPLAY] Figure deleted.\n'); catch ME_delete, warning('TacticalReplay:DeleteWarn', 'Error deleting figure: %s', ME_delete.message); end; fprintf('[REPLAY] Tactical replay mode closed.\n'); end %

 end % END OF FUNCTION tactical_replay_mode %

 % =========================================================================
 % LOCAL HELPER FUNCTIONS (for tactical_replay_mode)
 % =========================================================================
 % (Helpers: set_figure_style_helper_local, set_axes_style_helper_local,
 %  apply_theme_defaults_replay_local, ifelse remain the same as v1.5
 %  EXCEPT for plot_terrain_raytrace_helper_local)

 % --- Helper: Plot Terrain --- (Copied from Ray Trace for consistency) ---
 function plot_terrain_raytrace_helper_local(ax, terrain_in, lon_range, lat_range, theme, M2FT_in) % Use terrain_in % PASS M2FT_in %
      % Define M2FT locally or ensure it's available
      M2FT = M2FT_in; % Use passed-in value % V1.5 FIX

      % Check if terrain object and data seem valid before proceeding
      % V1.6 FIX: Use terrain_in here, check data_loaded field
      if ~isstruct(terrain_in) || ~isfield(terrain_in, 'data_loaded') || ~terrain_in.data_loaded || ...
         ~isfield(terrain_in,'get_altitude') || ~isa(terrain_in.get_altitude,'function_handle') % Check handle too %
           % warning('TacticalReplay:InvalidTerrainInputHelper','Terrain data provided to plot helper is invalid or empty.'); % Can be noisy
           return; % Exit if terrain data is not valid %
      end %

      grid_size = 75; % Resolution for terrain plot %
      lon_vec = linspace(lon_range(1), lon_range(2), grid_size); lat_vec = linspace(lat_range(1), lat_range(2), grid_size); %
      [LON, LAT] = meshgrid(lon_vec, lat_vec); ELEV_M = nan(size(LON)); % Initialize with NaN %

      % Get elevation for each grid point using the terrain object's function handle
      for i = 1:grid_size %
          for j = 1:grid_size %
               try
                   % V1.6 FIX: Call the function handle stored in the struct field
                   ELEV_M(i,j) = terrain_in.get_altitude(LAT(i,j), LON(i,j));
               catch ME_terr_lookup %
                    persistent terrain_lookup_warning_shown_plot;
                    if isempty(terrain_lookup_warning_shown_plot) % Check if warning shown before %
                       warning('tactical_replay_mode:TerrainLookupWarnPlot', 'Terrain lookup failed during plotting (further warnings suppressed): %s', ME_terr_lookup.message); % Added ID %
                       terrain_lookup_warning_shown_plot = true; % Set flag %
                    end %
                    % ELEV_M(i,j) remains NaN (initialized state) %
               end %
          end %
      end %
      ELEV_FT = ELEV_M * M2FT; % Convert meters to feet for plotting % Use LOCAL M2FT %

      valid_elevs = ELEV_FT(~isnan(ELEV_FT)); % Get valid elevation data %
      if isempty(valid_elevs) || range(valid_elevs) < 1 % Check if no valid data or range is negligible %
           mean_elev_ft = mean(valid_elevs, 'all'); if isnan(mean_elev_ft), mean_elev_ft = 0; end % Handle empty valid_elevs case %
           surf(ax, LON, LAT, ones(size(LON))*mean_elev_ft, 'FaceColor', theme.terrainFlatColor, 'EdgeColor', 'none', 'FaceAlpha', theme.terrainAlpha); % Plot flat surface %
      else % Plot the actual terrain surface %
           surf(ax, LON, LAT, ELEV_FT, 'EdgeColor', 'none', 'FaceAlpha', theme.terrainAlpha); % Plot surface %
           try colormap(ax, theme.terrainColormap); catch; colormap(ax, 'terrain'); end % Apply colormap, fallback %
           try material(ax,'dull'); catch; end % Apply material, ignore error %
      end %
 end %

 % --- Helper: Apply Figure Style ---
 function set_figure_style_helper_local(fig_handle, theme, is_cyberpunk) %
     if is_cyberpunk, set(fig_handle, 'Color', theme.bgColor); %
     else set(fig_handle, 'Color', [0.94 0.94 0.94]); end %
     set(fig_handle, 'Visible', 'on'); %
 end %
 % --- Helper: Apply Axes Style ---
 function set_axes_style_helper_local(ax_handle, theme, is_cyberpunk) %
     set(ax_handle, 'FontSize', theme.fontSize, 'FontName', theme.fontName); %
     if is_cyberpunk, set(ax_handle, 'Color',theme.axesColor,'XColor',theme.axesTickColor,'YColor',theme.axesTickColor,'ZColor',theme.axesTickColor,'GridColor',theme.gridColor,'GridAlpha',0.3); %
     else set(ax_handle, 'Color','w','XColor','k','YColor','k','ZColor','k','GridColor',[0.15 0.15 0.15],'GridAlpha',0.15); end %
 end %
 % --- Helper: Apply Theme Defaults ---
 function theme_out = apply_theme_defaults_replay_local(theme_in) %
     theme_out = theme_in; %
     defaults = struct('bgColor',[0.94 0.94 0.94],'axesColor',[1 1 1],'axesTickColor',[0 0 0],'gridColor',[0.8 0.8 0.8],'textColor',[0 0 0],'textColorEmphasis',[1 0 0],'titleColor',[0 0 0],'labelColor',[0 0 0],'fontSize',10,'fontName','Helvetica','lineColors',{{[0 0.4470 0.7410]}},'highlightColor',[0.8500 0.3250 0.0980],'markerColorWaypoint',[0.4940 0.1840 0.5560],'markerColorGroundStation',[0.4660 0.6740 0.1880],'statusOkColor',[0 1 0],'statusWarnColor',[1 0.5 0],'statusFailColor',[1 0 0],'statusUnknownColor',[0.5 0.5 0.5],'markerSize',6,'lineWidth',1.5,'terrainColormap', 'terrain', 'terrainFlatColor',[0.8 0.8 0.8],'terrainAlpha',0.6); %
     fields_def = fieldnames(defaults); %
     for i=1:length(fields_def), if ~isfield(theme_out, fields_def{i}), theme_out.(fields_def{i})=defaults.(fields_def{i}); end; end %
 end %
 % --- Helper: If-Else ---
 function out = ifelse(condition, true_val, false_val), if condition, out = true_val; else out = false_val; end; end %