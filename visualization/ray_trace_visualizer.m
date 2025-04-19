% =========================================================================
% RAY TRACE VISUALIZER (FINAL - Corrected v1.3) % <-- Version Update
% Version: FINAL 1.3
%
% Description:
%   Generates a *static* 3D plot showing straight-line rays (representing
%   Line-of-Sight paths) from the aircraft's position at a specific time
%   step to each configured ground station. Rays are color-coded based on
%   the calculated LOS status (Clear, Aircraft Masked, Terrain Blocked)
%   obtained from the processed RF simulation results. Optionally overlays
%   terrain for better visualization of potential blockages. Helps debug
%   LOS issues.
%
% Usage:
%   Called from `main_simulation.m` after simulation results are processed.
%   Can be called for a specific time index (e.g., end time or points of interest).
%   `ray_trace_visualizer(config, tspi_data, processed_rf_data, terrain, simulation_metadata, time_idx);`
%
% Inputs:
%   config              - Main configuration struct.
%   tspi_data           - NxM matrix of TSPI data.
%   processed_rf_data   - Structured RF data (output of post_process_rf_log_helper).
%                         Used to get LOS status (terrain_blocked, aircraft_masked, status)
%                         for the specified `time_idx`.
%   terrain             - Terrain model struct (output of `terrain_model.m`).
%                         Must contain the `get_altitude` function handle.
%   simulation_metadata - Metadata struct (contains `tspi_headers`).
%   time_idx            - (Optional) Scalar index into `tspi_data` and `processed_rf_data`
%                         specifying the time step to visualize. Defaults to the last step.
%
% Outputs:
%   - Displays a 3D plot figure.
%   - Saves a snapshot of the plot to the `figures/` directory if snapshot export is enabled.
%
% Patch 4 Integration:
%   - Includes fixes for robust terrain plotting, handling potentially flat terrain.
%   - Adjusts viewpoint and axis limits dynamically to frame the relevant scene.
%   - Uses status information from `processed_rf_data` to color rays accurately.
%
% Maintainer: [Your Name/Team]
% Last Updated: [2025-04-06] - Fixed terrain.get_altitude call (use function handle) (v1.3)
%                           - Refactored RF ray plotting logic similar to tactical replay (v1.2)
%                           - Fixed M2FT scope in terrain helper (v1.2)
% =========================================================================
function ray_trace_visualizer(config, tspi_data, processed_rf_data, terrain, simulation_metadata, time_idx)

    % --- Input & Config Checks ---
    % Check if this visualization is enabled
    if ~isfield(config.visualization,'ENABLE_RAY_TRACE_VISUALIZER') || ~config.visualization.ENABLE_RAY_TRACE_VISUALIZER % Added isfield %
        fprintf('[RAY] Ray trace visualization disabled in configuration.\n');
        return;
    end
    % Check if necessary data is available
    rf_data_available = ~isempty(fieldnames(processed_rf_data));
    if ~rf_data_available
        warning('[RAY] Processed RF data is empty. Ray coloring will indicate Unknown status.');
    end
    if isempty(tspi_data)
        warning('[RAY] TSPI data is empty. Cannot generate ray trace plot.');
        return;
    end
    % Validate or default the time index
    if nargin < 6 || isempty(time_idx) || ~isscalar(time_idx) || time_idx > size(tspi_data, 1) || time_idx < 1 % Added isscalar %
        time_idx = size(tspi_data, 1); % Default to last time step
        if time_idx < 1, warning('[RAY] No valid time steps in TSPI data.'); return; end
        fprintf('[RAY] Using default time index (last): %d\n', time_idx); % Inform user %
    end
    time_idx = round(time_idx); % Ensure integer index

    % --- V1.3 Check: Validate Terrain Struct ---
    if ~isstruct(terrain) || ~isfield(terrain,'get_altitude') || ~isa(terrain.get_altitude, 'function_handle')
         warning('RayTrace:InvalidTerrainInput','Input terrain argument is not a valid struct with get_altitude function handle. Terrain plotting disabled.');
         terrain.data_loaded = false; % Prevent terrain plotting attempt
    end
    % --- End V1.3 Check ---

    fprintf('[RAY] Generating Ray Trace visualization for time index %d (T=%.1f s)...\n', time_idx, tspi_data(time_idx, 1));

    % --- Load Theme ---
    theme = struct(); is_cyberpunk = false; % Initialize defaults
    if isfield(config.visualization,'PLOT_STYLE') && strcmpi(config.visualization.PLOT_STYLE, 'cyberpunk') % Added isfield %
        try
            theme = cyberpunk_theme(); % Load theme settings
            is_cyberpunk = true;
        catch ME_theme
            warning('[RAY] Failed to load cyberpunk theme: %s.', ME_theme.message);
        end
    end
    % Apply default theme values for robustness
    theme = apply_theme_defaults_local(theme);

    % --- Create Figure & Axes ---
    fig_pos = config.visualization.FIGURE_POSITION + [50 -50 0 0]; % Offset slightly from replay %
    fig_h = figure('Name', 'Ray Trace Visualization', 'Position', fig_pos); % Use variable %
    set_figure_style_helper_local(fig_h, theme, is_cyberpunk); % Apply figure style
    ax = axes('Parent', fig_h); % Create axes
    hold(ax, 'on'); grid(ax, 'on'); view(ax, 3); % Set 3D view, hold for multiple plots
    set_axes_style_helper_local(ax, theme, is_cyberpunk); % Apply axes style

    % --- Extract Aircraft State at Specified Time Index ---
    M2FT = 1 / 0.3048; % Define M2FT conversion factor needed here %
    try
        hdrs = lower(simulation_metadata.tspi_headers); % Use headers for column lookup
        lat_col = find(strcmp(hdrs, 'lat_deg'), 1); lon_col = find(strcmp(hdrs, 'lon_deg'), 1); alt_col = find(strcmp(hdrs, 'alt_m_msl'), 1);
        if isempty(lat_col)||isempty(lon_col)||isempty(alt_col), error('Missing Lat/Lon/Alt headers in TSPI.'); end
        ac_lat    = tspi_data(time_idx, lat_col);
        ac_lon    = tspi_data(time_idx, lon_col);
        ac_alt_m  = tspi_data(time_idx, alt_col);
        ac_alt_ft = ac_alt_m * M2FT; % Convert altitude to feet for display % Use M2FT %
        time_s    = tspi_data(time_idx, 1); % Time of this snapshot
    catch ME_tspi
        warning('[RAY] Failed to extract state from TSPI data at index %d: %s. Cannot generate plot.', time_idx, ME_tspi.message);
        if isgraphics(fig_h), try close(fig_h); catch; end; end % Close figure if created, check handle %
        return;
    end

    % --- Plot Aircraft Position ---
    h_ac = plot3(ax, ac_lon, ac_lat, ac_alt_ft, 'p', 'MarkerSize', theme.markerSize+4, ...
                 'Color', theme.highlightColor, 'MarkerFaceColor', theme.highlightColor); % Prominent AC marker
    text(ax, ac_lon, ac_lat, ac_alt_ft, ' Aircraft', 'Color', theme.textColorEmphasis, ...
         'FontSize', theme.fontSize, 'FontName', theme.fontName, 'VerticalAlignment', 'bottom'); % Label

    % --- Plot Ground Stations ---
    gs_plots = []; gs_labels = {}; % Store handles/labels for legend
    num_gs = 0; % Initialize ground station counter %
    if isfield(config.rf, 'ground_stations') && ~isempty(config.rf.ground_stations) && iscell(config.rf.ground_stations) % Added iscell %
        num_gs = length(config.rf.ground_stations); % Get number of configured ground stations %
        for i_gs = 1:num_gs % Use num_gs %
            gs_cfg    = config.rf.ground_stations{i_gs};
            if ~isfield(gs_cfg,'lon_deg') || ~isfield(gs_cfg,'lat_deg') || ~isfield(gs_cfg,'alt_m_msl') || ~isfield(gs_cfg,'id') % Check config %
                warning('RayTrace:GroundStationConfigWarn', 'Ground station %d is missing required fields (lon_deg, lat_deg, alt_m_msl, id). Skipping.', i_gs); % Added ID %
                continue; % Skip this ground station if config is bad %
            end %
            gs_alt_ft = gs_cfg.alt_m_msl * M2FT; % Convert to feet % Use M2FT %
            h_gs      = plot3(ax, gs_cfg.lon_deg, gs_cfg.lat_deg, gs_alt_ft, 's', 'MarkerSize', theme.markerSize+2, ...
                              'Color', theme.markerColorGroundStation, 'MarkerFaceColor', theme.markerColorGroundStation); % Square marker
            text(ax, gs_cfg.lon_deg, gs_cfg.lat_deg, gs_alt_ft, sprintf(' %s', gs_cfg.id), ...
                 'Color', theme.textColor, 'FontSize', theme.fontSize-1, 'FontName', theme.fontName, ...
                 'VerticalAlignment', 'bottom'); % Label
            if isempty(gs_plots), gs_plots = h_gs; gs_labels = {'Ground Station'}; end % Assign first valid plot %
        end
    else
        warning('[RAY] No ground stations defined in config.rf.ground_stations.'); % Add warning %
    end

    % ============================================================== %
    % --- V1.3: RAY PLOTTING LOGIC (Refactored in V1.2) ---
    % ============================================================== %
    % Store handles for legend entries (one per status type)
    ray_plots     = gobjects(4,1); % OK, Masked, Terrain, Unknown/Skip
    ray_labels    = {'Clear LOS', 'Aircraft Masked', 'Terrain Blocked', 'Unknown/Skipped'};
    plots_created = false(4,1);    % Track if a plot handle has been created for each status

    if rf_data_available && num_gs > 0 % Check if processed RF data exists AND ground stations are configured %
        try % Wrap the entire RF access and plotting logic %
            % Iterate through each CONFIGURED ground station
            for i_gs_ray = 1:num_gs
                 % Get the config for this GS (handle potential errors if num_gs doesn't match length)
                 if i_gs_ray > length(config.rf.ground_stations), break; end
                 gs_cfg_ray = config.rf.ground_stations{i_gs_ray}; %
                 if ~isfield(gs_cfg_ray,'lon_deg') || ~isfield(gs_cfg_ray,'lat_deg') || ~isfield(gs_cfg_ray,'alt_m_msl') || ~isfield(gs_cfg_ray,'id') % Check again %
                     continue; % Skip this ground station if config is bad %
                 end %
                 target_rx_id_ray = gs_cfg_ray.id; % ID of the current ground station (acting as Rx) %
                 rx_lon_ray = gs_cfg_ray.lon_deg;   % Lon of this GS %
                 rx_lat_ray = gs_cfg_ray.lat_deg;   % Lat of this GS %
                 rx_alt_ft_ray = gs_cfg_ray.alt_m_msl * M2FT; % Alt (ft) of this GS % Use M2FT %

                 % --- Determine Ray Origin ---
                 tx_pos_llh_origin = [ac_lat, ac_lon, ac_alt_m]; % Using AC CG Lat/Lon/Alt(m)
                 tx_alt_ft_origin  = ac_alt_ft;                 % Alt in feet for plotting

                 % --- Search processed_rf_data for a link TO this receiver at the target time step ---
                 found_link_data_ray = []; % Initialize flag/data holder %
                 tx_ids_ray = fieldnames(processed_rf_data); % Get all transmitter IDs from the data %
                 for i_tx_ray = 1:length(tx_ids_ray) % Loop through transmitters found in data %
                     current_tx_id_ray = tx_ids_ray{i_tx_ray}; % Get the current Tx ID string %
                     if isfield(processed_rf_data.(current_tx_id_ray), target_rx_id_ray) % Check if Rx field exists for this Tx %
                         link_data_candidate = processed_rf_data.(current_tx_id_ray).(target_rx_id_ray); % Get the data struct %
                         % --- Validation Logic (same as Tactical Replay v1.5/1.6) ---
                         is_valid_struct = isstruct(link_data_candidate);
                         has_status = is_valid_struct && isfield(link_data_candidate,'status') && iscell(link_data_candidate.status) && ~isempty(link_data_candidate.status);
                         has_terrain = is_valid_struct && isfield(link_data_candidate,'terrain_blocked') && islogical(link_data_candidate.terrain_blocked) && ~isempty(link_data_candidate.terrain_blocked);
                         has_mask = is_valid_struct && isfield(link_data_candidate,'aircraft_masked') && islogical(link_data_candidate.aircraft_masked) && ~isempty(link_data_candidate.aircraft_masked);
                         idx_ok_status = has_status && time_idx <= length(link_data_candidate.status);
                         idx_ok_terrain = has_terrain && time_idx <= length(link_data_candidate.terrain_blocked);
                         idx_ok_mask = has_mask && time_idx <= length(link_data_candidate.aircraft_masked);
                         if is_valid_struct && has_status && has_terrain && has_mask && idx_ok_status && idx_ok_terrain && idx_ok_mask % Check ALL conditions %
                             found_link_data_ray = link_data_candidate; % Store the valid data struct %
                             break; % Stop searching Tx's for this GS, use the first one found %
                         end % --- End Validation ---
                     end % End if receiver field exists %
                 end % End loop through transmitters %

                 % --- Determine Ray Color based on FOUND link data (or default) ---
                 ray_color = theme.statusUnknownColor; ray_type = 4; % Defaults %
                 if ~isempty(found_link_data_ray) % If we found valid data for this GS receiver %
                     current_status_ray     = found_link_data_ray.status{time_idx};
                     is_terrain_blocked_ray = found_link_data_ray.terrain_blocked(time_idx);
                     is_aircraft_masked_ray = found_link_data_ray.aircraft_masked(time_idx);
                     if strcmp(current_status_ray, 'SKIP'), ray_color = theme.statusUnknownColor; ray_type = 4;
                     elseif is_terrain_blocked_ray, ray_color = theme.statusFailColor; ray_type = 3;
                     elseif is_aircraft_masked_ray, ray_color = theme.statusWarnColor; ray_type = 2;
                     elseif strcmp(current_status_ray, 'OK'), ray_color = theme.statusOkColor; ray_type = 1;
                     elseif strcmp(current_status_ray, 'FAIL'), ray_color = theme.statusFailColor; ray_type = 3;
                     end
                 end % End if data found %

                 % --- Plot the Ray for THIS ground station ---
                 h_ray = plot3(ax, [tx_pos_llh_origin(2), rx_lon_ray], [tx_pos_llh_origin(1), rx_lat_ray], [tx_alt_ft_origin, rx_alt_ft_ray], ...
                                   '-', 'Color', [ray_color, 0.8], 'LineWidth', theme.lineWidth - 0.5); %

                 % --- Store Handle for Legend (only the first of each type) ---
                 if ~plots_created(ray_type), ray_plots(ray_type) = h_ray; plots_created(ray_type) = true; end
            end % End loop through ground stations (i_gs_ray) %
        catch ME_ray_plot % Catch errors during RF data access or plotting %
            warning('[RAY] Error processing or plotting rays: %s', ME_ray_plot.message); % More generic message %
        end % End try-catch for RF processing %
    else % No RF data available or no ground stations %
         % (Code to draw default gray lines remains the same as v1.2)
         % ... [Code identical to v1.2] ...
         if ~rf_data_available, warning('[RAY] No processed RF data available to determine ray status/color.'); end
         if num_gs > 0
             for i_gs = 1:num_gs
                  if i_gs > length(config.rf.ground_stations), break; end
                  gs_cfg = config.rf.ground_stations{i_gs};
                  if ~isfield(gs_cfg,'lon_deg') || ~isfield(gs_cfg,'lat_deg') || ~isfield(gs_cfg,'alt_m_msl'), continue; end % Skip invalid config %
                  gs_alt_ft = gs_cfg.alt_m_msl*M2FT; % Use M2FT %
                  plot3(ax, [ac_lon, gs_cfg.lon_deg], [ac_lat, gs_cfg.lat_deg], [ac_alt_ft, gs_alt_ft], ...
                        ':', 'Color', [0.5 0.5 0.5 0.5], 'LineWidth', 1); % Dim dashed gray line
             end
         end
         ray_labels = {}; ray_plots = gobjects(4,1); plots_created = false(4,1); % Reset legend vars %

    end % End if rf_data_available and num_gs > 0 %
    % ============================================================== %
    % --- END V1.3 RAY PLOTTING LOGIC ---
    % ============================================================== %

    % --- Plot Terrain (Using Helper Function) ---
    if isfield(terrain, 'data_loaded') && terrain.data_loaded && ... % Check terrain struct validity FIRST %
       isfield(config.visualization, 'ENABLE_TERRAIN_VIEW') && config.visualization.ENABLE_TERRAIN_VIEW % Added isfield %
         try
             fprintf('  Plotting terrain for Ray Trace...\n');
             lon_lims = xlim(ax); lat_lims = ylim(ax); % Get initial limits set by AC/GS
             % PASS M2FT and the terrain struct to the helper function
             plot_terrain_raytrace_helper_local(ax, terrain, lon_lims, lat_lims, theme, M2FT); % PASS terrain struct %
         catch ME_terr_plot
             warning('Failed to plot terrain in Ray Trace: %s', ME_terr_plot.message);
         end
    end
    hold(ax, 'off'); % Release hold on axes

    % --- Adjust Viewpoint & Axes Limits ---
    % (Axis limit calculation code remains the same as v1.2)
    % ... [Code identical to v1.2] ...
    all_lons = [ac_lon]; all_lats = [ac_lat]; all_alts_ft = [ac_alt_ft]; %
    if num_gs > 0 %
         gs_lons = []; gs_lats = []; gs_alts = []; %
         try %
             gs_lons_cell = cellfun(@(x) ifelse(isfield(x,'lon_deg'), x.lon_deg, NaN), config.rf.ground_stations, 'UniformOutput', false); gs_lons = cell2mat(gs_lons_cell); %
             gs_lats_cell = cellfun(@(x) ifelse(isfield(x,'lat_deg'), x.lat_deg, NaN), config.rf.ground_stations, 'UniformOutput', false); gs_lats = cell2mat(gs_lats_cell); %
             gs_alts_cell = cellfun(@(x) ifelse(isfield(x,'alt_m_msl'), x.alt_m_msl*M2FT, NaN), config.rf.ground_stations, 'UniformOutput', false); gs_alts = cell2mat(gs_alts_cell); % Use M2FT %
             gs_lons = gs_lons(~isnan(gs_lons)); gs_lats = gs_lats(~isnan(gs_lats)); gs_alts = gs_alts(~isnan(gs_alts)); %
             all_lons = [all_lons; gs_lons(:)]; all_lats = [all_lats; gs_lats(:)]; all_alts_ft = [all_alts_ft; gs_alts(:)]; %
         catch ME_gs_bounds, warning('RayTrace:GSBoundsError', 'Could not get all ground station bounds for axis limits: %s', ME_gs_bounds.message); end %
    end %
    terrain_surf = findobj(ax, 'Type', 'surface');
    if ~isempty(terrain_surf), z_data_terr = terrain_surf.ZData; z_data_terr = z_data_terr(~isnan(z_data_terr)); if ~isempty(z_data_terr), all_alts_ft = [all_alts_ft; min(z_data_terr); max(z_data_terr)]; end; end %
    min_lon=min(all_lons);max_lon=max(all_lons);min_lat=min(all_lats);max_lat=max(all_lats);min_alt=min(all_alts_ft);max_alt=max(all_alts_ft);
    range_lon=max(max_lon-min_lon,0.01); range_lat=max(max_lat-min_lat,0.01); range_alt=max(max_alt-min_alt,100);
    pad_lon=range_lon*0.1; pad_lat=range_lat*0.1; pad_alt=range_alt*0.1+100;
    xlim(ax, [min_lon-pad_lon, max_lon+pad_lon]); ylim(ax, [min_lat-pad_lat, max_lat+pad_lat]); zlim(ax, [min_alt-pad_alt, max_alt+pad_alt]);
    axis(ax, 'vis3d'); view(ax, -35, 30); try camva(ax, 8); catch; end %

    % --- Add Title & Legend ---
    title(ax, sprintf('Ray Trace Visualization (T = %.1f s)', time_s), 'Color', theme.titleColor, 'FontSize', theme.fontSize+1, 'FontName', theme.fontName);
    xlabel(ax, 'Longitude (deg)', 'Color', theme.labelColor, 'FontName', theme.fontName);
    ylabel(ax, 'Latitude (deg)', 'Color', theme.labelColor, 'FontName', theme.fontName);
    zlabel(ax, 'Altitude (ft MSL)', 'Color', theme.labelColor, 'FontName', theme.fontName);
    valid_ray_plots = ray_plots(plots_created); valid_ray_labels = ray_labels(plots_created); %
    valid_plots = [h_ac; gs_plots(:); valid_ray_plots(:)]; valid_labels = [{'Aircraft'}; gs_labels(:); valid_ray_labels(:)]; %
    if ~isempty(valid_plots), lgd = legend(ax, valid_plots, valid_labels, 'Location', 'bestoutside'); if is_cyberpunk, set(lgd, 'TextColor', theme.textColor, 'Color', [theme.axesColor, 0.8], 'EdgeColor', theme.gridColor); end; end %

    % --- Save Figure Snapshot ---
    if isfield(config.output,'ENABLE_SNAPSHOT_EXPORT') && config.output.ENABLE_SNAPSHOT_EXPORT % Added isfield check %
        snapshot_generator(fig_h, 'ray_trace', config, sprintf('T%.1fs', time_s));
    end

    fprintf('[RAY] Ray trace visualization generation complete.\n');

end % END OF FUNCTION ray_trace_visualizer


% =========================================================================
% LOCAL HELPER FUNCTIONS for Ray Trace Visualizer
% =========================================================================

% --- Helper to Plot Terrain ---
function plot_terrain_raytrace_helper_local(ax, terrain_in, lon_range, lat_range, theme, M2FT_in) % Use terrain_in % PASS M2FT_in %
    % Plots terrain surface using data from terrain model object.
    % Handles potentially flat terrain based on elevation range.

    % Define M2FT locally or ensure it's available
    M2FT = M2FT_in; % Use passed-in value % V1.2 FIX

     % Check if terrain object and data seem valid before proceeding
     % V1.3 FIX: Use terrain_in here, check data_loaded field and function handle
      if ~isstruct(terrain_in) || ~isfield(terrain_in, 'data_loaded') || ~terrain_in.data_loaded || ...
         ~isfield(terrain_in,'get_altitude') || ~isa(terrain_in.get_altitude,'function_handle') % Check handle too %
           % warning('RayTrace:InvalidTerrainInputHelper','Terrain data provided to plot helper is invalid or empty.'); % Can be noisy
           return; % Exit if terrain data is not valid %
      end %

     grid_size = 300 %75; % Resolution for terrain plot %
     lon_vec = linspace(lon_range(1), lon_range(2), grid_size); lat_vec = linspace(lat_range(1), lat_range(2), grid_size); %
     [LON, LAT] = meshgrid(lon_vec, lat_vec); ELEV_M = nan(size(LON)); % Preallocate elevation matrix with NaN %

     % Get elevation for each grid point using the terrain object's function handle
     for i = 1:grid_size
         for j = 1:grid_size
             try
                 % V1.3 FIX: Call the function handle stored in the struct field
                 ELEV_M(i,j) = terrain_in.get_altitude(LAT(i,j), LON(i,j));
             catch ME_getalt
                  persistent terrain_lookup_warning_shown_raytrace; % Use persistent variable %
                  if isempty(terrain_lookup_warning_shown_raytrace) % Check if warning shown before %
                      warning('RayTrace:TerrainLookupWarnPlot', 'Terrain lookup failed during plotting (further warnings suppressed): %s',ME_getalt.message); % Added ID %
                      terrain_lookup_warning_shown_raytrace = true; % Set flag %
                  end %
                  % ELEV_M(i,j) remains NaN %
             end
         end
     end
     ELEV_FT = ELEV_M * M2FT; % Convert elevation to feet for plotting % Use LOCAL M2FT %

     % --- Patch 4 Fix / V1.3 Refinement: Handle Flat Terrain ---
     valid_elevs = ELEV_FT(~isnan(ELEV_FT)); % Get only valid (non-NaN) elevations %
     if isempty(valid_elevs) || range(valid_elevs) < 1.0 % Check if no valid points or range < threshold %
          fprintf('  Terrain elevation range < 1 ft or no valid data. Plotting as flat surface.\n'); % Update message %
          mean_elev_ft = mean(valid_elevs, 'all'); if isnan(mean_elev_ft), mean_elev_ft = 0; end % Handle case where valid_elevs is empty %
          surf(ax, LON, LAT, ones(size(LON))*mean_elev_ft, ...
               'FaceColor', theme.terrainFlatColor, 'EdgeColor', 'none', 'FaceAlpha', theme.terrainAlpha);
     else
          % Plot terrain surface using surf (nicer shading than mesh)
          surf(ax, LON, LAT, ELEV_FT, 'EdgeColor', 'none', 'FaceAlpha', theme.terrainAlpha);
          try colormap(ax, theme.terrainColormap); catch; colormap(ax, 'terrain'); end % Apply colormap, fallback %
          try material(ax,'dull'); catch; end % Adjust lighting properties, ignore error %
     end
     fprintf('  Terrain plotted.\n');
end

% --- Helper: Apply Figure Style ---
function set_figure_style_helper_local(fig_handle, theme, is_cyberpunk) % Same as v1.2 %
    if is_cyberpunk, set(fig_handle, 'Color', theme.bgColor);
    else set(fig_handle, 'Color', [0.94 0.94 0.94]); end
    set(fig_handle, 'Visible', 'on'); %
end

% --- Helper: Apply Axes Style ---
function set_axes_style_helper_local(ax_handle, theme, is_cyberpunk) % Same as v1.2 %
    set(ax_handle, 'FontSize', theme.fontSize, 'FontName', theme.fontName);
    if is_cyberpunk, set(ax_handle, 'Color',theme.axesColor,'XColor',theme.axesTickColor,'YColor',theme.axesTickColor,'ZColor',theme.axesTickColor,'GridColor',theme.gridColor,'GridAlpha',0.3);
    else set(ax_handle, 'Color','w','XColor','k','YColor','k','ZColor','k','GridColor',[0.15 0.15 0.15],'GridAlpha',0.15); end
end

% --- Helper: Apply Theme Defaults ---
function theme_out = apply_theme_defaults_local(theme_in) % Same as v1.2 %
    theme_out = theme_in;
    defaults = struct('bgColor',[0.94 0.94 0.94],'axesColor',[1 1 1],'axesTickColor',[0 0 0],'gridColor',[0.8 0.8 0.8],'textColor',[0 0 0],'textColorEmphasis',[1 0 0],'titleColor',[0 0 0],'labelColor',[0 0 0],'fontSize',10,'fontName','Helvetica','lineColors',{{[0 0.4470 0.7410]}},'highlightColor',[0.8500 0.3250 0.0980],'markerColorWaypoint',[0.4940 0.1840 0.5560],'markerColorGroundStation',[0.4660 0.6740 0.1880],'statusOkColor',[0 1 0],'statusWarnColor',[1 0.5 0],'statusFailColor',[1 0 0],'statusUnknownColor',[0.5 0.5 0.5],'markerSize',6,'lineWidth',1.5,'terrainColormap', 'terrain', 'terrainFlatColor',[0.8 0.8 0.8],'terrainAlpha',0.6); %
    fields_def = fieldnames(defaults);
    for i = 1:length(fields_def), if ~isfield(theme_out, fields_def{i}), theme_out.(fields_def{i}) = defaults.(fields_def{i}); end; end
end

 % --- Helper: If-Else for cellfun/arrayfun ---
 function out = ifelse(condition, true_val, false_val) % Same as v1.2 %
     if condition, out = true_val; else out = false_val; end %
 end %