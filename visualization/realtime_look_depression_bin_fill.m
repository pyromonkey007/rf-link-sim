% =========================================================================
% REAL-TIME LOOK/DEPRESSION ANGLE BIN FILL PLOT & DISPLAY (FINAL - Re-Commented)
% Version: FINAL 1.1 (Based on User-Provided Structure)
%
% Description:
%   Provides functions to initialize (`realtime_look_depression_init`) and
%   update (`realtime_look_depression_update`) a real-time plot displayed
%   during the simulation. This plot consists of two main parts:
%
%   1. Bin Fill Heatmap: A 2D heatmap (using `imagesc`) visualizing the
%      distribution of Look Angle (Azimuth) vs. Depression Angle (Elevation)
%      occurrences relative to a configured reference point (e.g., radar site).
%      The color intensity of each bin indicates the number of simulation time
%      steps the aircraft spent pointing within that specific angle bin, but only
%      counts data collected between specified start and end waypoints configured
%      by `REALTIME_LOOKDEP_START_WP` and `REALTIME_LOOKDEP_END_WP`.
%
%   2. Text Display Panel: A section displaying current, key flight parameters
%      updated in real-time, including Altitude, Calibrated Airspeed (KCAS),
%      Heading, Pitch, Roll, current target Waypoint index, and estimated
%      Time-To-Go (TTG) to that waypoint.
%
%   Applies standard or cyberpunk visual themes based on the main configuration.
%
% Usage:
%   - Initialization (called ONCE before simulation loop, e.g., in `aircraft_dynamics_model.m`):
%     `handles = realtime_look_depression_init(config);`
%     This creates the figure, axes, graphics objects (image, colorbar, text),
%     initializes data structures (bin counts), and returns a `handles` structure
%     containing references to these objects.
%
%   - Update (called REPEATEDLY inside simulation loop, rate-limited):
%     `realtime_look_depression_update(handles, look_deg, depr_deg, display_data);`
%     This function takes the `handles` structure, the current look/depression angles,
%     and a structure containing the current values for the text display panel. It
%     updates the heatmap bin counts, refreshes the `imagesc` plot, and updates the
%     text strings.
%
% Inputs to `init`:
%   config - The main configuration structure (primarily uses `config.visualization`
%            and potentially `config.radar` indirectly via the title).
% Outputs of `init`:
%   handles - Structure containing handles for graphics objects and bin data:
%             .fig          (Figure handle)
%             .ax_binfill   (Axes handle for heatmap)
%             .ax_text      (Axes handle for text panel)
%             .az_edges     (Bin edges for azimuth)
%             .el_edges     (Bin edges for elevation)
%             .bin_counts   (Matrix storing counts per bin)
%             .az_centers   (Center coordinates of azimuth bins)
%             .el_centers   (Center coordinates of elevation bins)
%             .h_image      (Handle to the `imagesc` object)
%             .h_colorbar   (Handle to the colorbar)
%             .text_handles (Array of handles to the text value objects)
%
% Inputs to `update`:
%   handles      - The structure returned by the `init` function.
%   look_deg     - Current calculated look angle (azimuth) in degrees.
%   depr_deg     - Current calculated depression angle (elevation) in degrees.
%   display_data - Structure containing current values for the text display panel:
%                  .alt_ft (Altitude in feet)
%                  .kcas   (Calibrated Airspeed in knots - requires tas_to_kcas)
%                  .hdg    (Heading in degrees)
%                  .pitch  (Pitch in degrees)
%                  .roll   (Roll in degrees)
%                  .wp     (Current target waypoint index)
%                  .ttg    (Estimated time-to-go to current target WP in seconds)
%
% Performance Note:
%   Updating graphical elements frequently within a tight simulation loop can
%   significantly impact MATLAB's performance. This implementation uses `imagesc`
%   and updates its `CData`, which is generally faster than recreating histogram
%   objects repeatedly. The actual update rate is controlled by the calling function
%   (`aircraft_dynamics_model`) using `config.visualization.REALTIME_LOOKDEP_UPDATE_RATE_HZ`
%   and the `drawnow limitrate` command. Higher update rates give smoother visuals
%   but slower simulation; lower rates improve simulation speed at the cost of choppy visuals.
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-04] - Re-commented based on user version
% =========================================================================


% -------------------------------------------------------------------------
% INITIALIZATION FUNCTION: Creates the figure and plot objects
% -------------------------------------------------------------------------
function handles = realtime_look_depression_init(config)
    % Creates the figure window, sets up axes for the heatmap and text display,
    % initializes the bin counting matrix, creates the imagesc object for the
    % heatmap, adds a colorbar, creates text objects for data display, and
    % returns handles to all necessary graphics objects and data structures.

    handles = struct(); % Initialize empty output structure

    % --- Get Plot Parameters from Configuration ---
    vis_cfg  = config.visualization;
    az_range = vis_cfg.REALTIME_LOOKDEP_AZ_RANGE; % e.g., [-180, 180] degrees
    el_range = vis_cfg.REALTIME_LOOKDEP_EL_RANGE; % e.g., [-90, 90] degrees
    az_bins  = round(vis_cfg.REALTIME_LOOKDEP_AZ_BINS);  % Number of bins along azimuth axis
    el_bins  = round(vis_cfg.REALTIME_LOOKDEP_EL_BINS);  % Number of bins along elevation axis
    cmap     = vis_cfg.REALTIME_LOOKDEP_COLORMAP; % Name of the colormap (e.g., 'hot', 'jet')
    wp_start = vis_cfg.REALTIME_LOOKDEP_START_WP; % Waypoint index to start collecting data
    wp_end   = vis_cfg.REALTIME_LOOKDEP_END_WP;   % Waypoint index to stop collecting data

    % --- Apply Theme ---
    theme = struct(); % Initialize empty theme struct
    is_cyberpunk = strcmpi(vis_cfg.PLOT_STYLE, 'cyberpunk');
    if is_cyberpunk
        try
            theme = cyberpunk_theme(); % Load theme settings from file
        catch ME_theme
             warning('Failed to load cyberpunk theme for RT plot: %s. Using standard defaults.', ME_theme.message);
             is_cyberpunk = false; % Fallback if theme file error
        end
    end
    % Apply default theme values if fields are missing (ensures script runs even with incomplete theme)
    theme = apply_theme_defaults_rt_local(theme);

    % --- Create Figure Window ---
    handles.fig = figure(...
        'Name',        'Real-Time Look/Depression Bin Fill', ...
        'NumberTitle', 'off', ... % Don't show "Figure 1" etc. in title bar
        'ToolBar',     'none', ... % Hide default toolbar
        'MenuBar',     'none');    % Hide default menu bar
    % Apply background color based on theme
    set_figure_style_helper_rt(handles.fig, theme, is_cyberpunk);
    % Optional: Set a default position and size for the figure window
    % set(handles.fig, 'Position', [x_position, y_position, width, height]);

    % --- Create Axes for Bin Fill Heatmap (Top portion) ---
    % Use subplot to divide the figure vertically (5 rows, 1 column, use rows 1-4)
    handles.ax_binfill = subplot(5, 1, 1:4, 'Parent', handles.fig);
    % Apply theme colors and styles to the axes
    set_axes_style_helper_rt(handles.ax_binfill, theme, is_cyberpunk);
    % Set plot title, including the waypoint range being visualized
    title(handles.ax_binfill, sprintf('Look/Depression Bin Fill (Waypoints %d-%d)', wp_start, wp_end), ...
          'Color', theme.titleColor, 'FontSize', theme.fontSize + 1, 'FontName', theme.fontName);
    % Set axis labels
    xlabel(handles.ax_binfill, 'Look Angle / Azimuth (deg)', 'Color', theme.labelColor, 'FontSize', theme.fontSize, 'FontName', theme.fontName);
    ylabel(handles.ax_binfill, 'Depression Angle / Elevation (deg)', 'Color', theme.labelColor, 'FontSize', theme.fontSize, 'FontName', theme.fontName);
    % Turn on grid and hold state (to allow adding colorbar later)
    grid(handles.ax_binfill, 'on');
    hold(handles.ax_binfill, 'on');

    % --- Initialize Bin Data Structure ---
    % Define the edges of the bins along each axis
    handles.az_edges = linspace(az_range(1), az_range(2), az_bins + 1);
    handles.el_edges = linspace(el_range(1), el_range(2), el_bins + 1);
    % Initialize the count matrix (Rows correspond to Elevation, Columns to Azimuth)
    handles.bin_counts = zeros(el_bins, az_bins);

    % --- Create Heatmap Object (using imagesc) ---
    % Calculate bin centers (needed for positioning the image correctly on axes)
    handles.az_centers = handles.az_edges(1:end-1) + diff(handles.az_edges)/2;
    handles.el_centers = handles.el_edges(1:end-1) + diff(handles.el_edges)/2;
    % Create the image object; CData will be updated in the update function.
    handles.h_image = imagesc(handles.ax_binfill, handles.az_centers, handles.el_centers, handles.bin_counts);
    % Ensure Y-axis (Elevation) increases upwards, which is standard for plots
    set(handles.ax_binfill, 'YDir', 'normal');
    % Apply the configured colormap to the axes
    colormap(handles.ax_binfill, cmap);
    % Set the axis limits to match the configured ranges
    xlim(handles.ax_binfill, az_range);
    ylim(handles.ax_binfill, el_range);
    % Make axis scaling tight around the data/limits
    axis(handles.ax_binfill, 'tight');
    % Optional: enforce equal aspect ratio if desired
    % axis(handles.ax_binfill, 'equal');

    % --- Add and Style Colorbar ---
    handles.h_colorbar = colorbar(handles.ax_binfill);
    ylabel(handles.h_colorbar, 'Counts (log scale)', 'Color', theme.labelColor, 'FontSize', theme.fontSize, 'FontName', theme.fontName);
    if is_cyberpunk
        set(handles.h_colorbar, 'Color', theme.axesTickColor); % Colorbar text/tick color
    end
    % Set initial color axis limits (will be updated dynamically)
    caxis(handles.ax_binfill, [0, 1]); % Start with 0 to 1, representing log10(1) to log10(10) approx

    % Release hold on heatmap axes
    hold(handles.ax_binfill, 'off');

    % --- Create Axes/Area for Text Display (Bottom portion) ---
    % Use bottom 1/5th of the figure for text display
    handles.ax_text = subplot(5, 1, 5, 'Parent', handles.fig);
    axis(handles.ax_text, 'off'); % Turn off axes lines, ticks, labels for this subplot
    if is_cyberpunk % Match figure background color for the text area
        set(handles.ax_text, 'Color', theme.bgColor);
    end

    % --- Create Text Objects for Data Display ---
    % Define positions (normalized units [0,1] within the text subplot axes)
    x_pos_lbl1 = 0.02; x_pos_val1 = 0.15; % Column 1 (Label, Value) X positions
    x_pos_lbl2 = 0.32; x_pos_val2 = 0.45; % Column 2
    x_pos_lbl3 = 0.62; x_pos_val3 = 0.75; % Column 3
    y_pos_row1 = 0.65;                    % Top row Y position
    y_pos_row2 = 0.25;                    % Bottom row Y position
    % Define common text properties for labels and values
    lbl_props = {'Color', theme.labelColor, 'FontSize', theme.fontSize, 'HorizontalAlignment', 'right', 'FontName', theme.fontName};
    val_props = {'Color', theme.textColor, 'FontSize', theme.fontSize, 'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'FontName', theme.fontName};

    % Preallocate array to store handles of the text objects that display *values*
    handles.text_handles = gobjects(6, 1);

    % Create Labels (static) and Value Placeholders (dynamic, store handle) - Row 1
    text(handles.ax_text, x_pos_lbl1, y_pos_row1, 'Alt (ft):',   lbl_props{:});
    handles.text_handles(1) = text(handles.ax_text, x_pos_val1, y_pos_row1, 'N/A', val_props{:}); % Alt value handle

    text(handles.ax_text, x_pos_lbl2, y_pos_row1, 'KCAS:',        lbl_props{:});
    handles.text_handles(2) = text(handles.ax_text, x_pos_val2, y_pos_row1, 'N/A', val_props{:}); % KCAS value handle

    text(handles.ax_text, x_pos_lbl3, y_pos_row1, 'Heading (°):', lbl_props{:});
    handles.text_handles(3) = text(handles.ax_text, x_pos_val3, y_pos_row1, 'N/A', val_props{:}); % Heading value handle

    % Create Labels and Value Placeholders - Row 2
    text(handles.ax_text, x_pos_lbl1, y_pos_row2, 'Pitch (°):',  lbl_props{:});
    handles.text_handles(4) = text(handles.ax_text, x_pos_val1, y_pos_row2, 'N/A', val_props{:}); % Pitch value handle

    text(handles.ax_text, x_pos_lbl2, y_pos_row2, 'Roll (°):',   lbl_props{:});
    handles.text_handles(5) = text(handles.ax_text, x_pos_val2, y_pos_row2, 'N/A', val_props{:}); % Roll value handle

    text(handles.ax_text, x_pos_lbl3, y_pos_row2, 'WP / TTG:',    lbl_props{:}); % Waypoint / Time To Go label
    handles.text_handles(6) = text(handles.ax_text, x_pos_val3, y_pos_row2, 'N/A', val_props{:}); % WP/TTG value handle

end % END FUNCTION realtime_look_depression_init


% -------------------------------------------------------------------------
% UPDATE FUNCTION: Updates plot and text with new data
% -------------------------------------------------------------------------
function realtime_look_depression_update(handles, look_deg, depr_deg, display_data)
    % Updates the bin fill heatmap counts and the text display values.
    % Called repeatedly from the simulation loop (e.g., aircraft_dynamics_model).

    % --- Input Handle Check ---
    % Ensure the passed handles structure and the figure window are still valid.
    % The user might close the figure window while the simulation is running.
    if isempty(handles) || ~isstruct(handles) || ~isfield(handles, 'fig') || ~ishandle(handles.fig)
        % Silently return if the figure doesn't exist anymore.
        return;
    end

    % --- Update Bin Fill Data and Heatmap ---
    try
        % Determine which bin the current look/depression angles fall into.
        % Use the pre-calculated bin edges. `find` returns the index of the bin.
        az_idx = find(look_deg >= handles.az_edges(1:end-1) & look_deg < handles.az_edges(2:end), 1, 'first');
        el_idx = find(depr_deg >= handles.el_edges(1:end-1) & depr_deg < handles.el_edges(2:end), 1, 'first');

        % Increment the count for the corresponding bin if the angles are within the plot range.
        if ~isempty(az_idx) && ~isempty(el_idx)
            handles.bin_counts(el_idx, az_idx) = handles.bin_counts(el_idx, az_idx) + 1;

            % --- Update Heatmap Display ---
            % Get the handle to the image object.
            h_img = handles.h_image;
            % Update the CData (color data) property with the new counts.
            % Using a log scale (log10(counts + 1)) helps visualize data with
            % large variations in counts (otherwise dense bins saturate the colormap).
            log_counts = log10(handles.bin_counts + 1); % Add 1 to avoid log10(0) = -Inf
            if ishandle(h_img) % Check if image handle is still valid
                set(h_img, 'CData', log_counts);
            end

            % Adjust the color axis limits dynamically based on the maximum count.
            % This ensures the colormap spans the current range of counts.
            max_log_count = max(log_counts(:));
            if max_log_count > 0 % Avoid setting limits if counts are still zero
                caxis(handles.ax_binfill, [0, max_log_count]);
                % Update the colorbar label if needed (usually only once)
                % ylabel(handles.h_colorbar, 'Counts (log scale)');
            else
                 caxis(handles.ax_binfill, [0, 1]); % Default limits when no counts yet
            end
        end % End if indices found

    catch ME_bin % Catch errors during binning/plotting update
         warning('Error updating bin fill plot: %s', ME_bin.message);
         % Optional: Display error details: disp(ME_bin.getReport);
    end

    % --- Update Text Display ---
    try
        % Get the handles for the text value objects.
        txt_h = handles.text_handles;
        % Update the 'String' property of each text object using sprintf for formatting.
        if ishandle(txt_h(1)), set(txt_h(1), 'String', sprintf('%.0f', display_data.alt_ft)); end
        if ishandle(txt_h(2)), set(txt_h(2), 'String', sprintf('%.0f', display_data.kcas)); end % Assumes .kcas exists
        if ishandle(txt_h(3)), set(txt_h(3), 'String', sprintf('%.1f', display_data.hdg)); end
        if ishandle(txt_h(4)), set(txt_h(4), 'String', sprintf('%.1f', display_data.pitch)); end
        if ishandle(txt_h(5)), set(txt_h(5), 'String', sprintf('%.1f', display_data.roll)); end
        % Format Time-To-Go (TTG) nicely, handling Inf or NaN.
        if isinf(display_data.ttg)
            ttg_str = 'Inf';
        elseif isnan(display_data.ttg)
            ttg_str = 'N/A';
        else
            ttg_str = sprintf('%.1f s', display_data.ttg); % Display with units
        end
        if ishandle(txt_h(6)), set(txt_h(6), 'String', sprintf('%d / %s', display_data.wp, ttg_str)); end

    catch ME_text % Catch errors during text update
        warning('Error updating text display: %s', ME_text.message);
    end

    % --- Refresh Plot Window ---
    % `drawnow limitrate` updates the figure window to show the changes made above.
    % 'limitrate' option helps prevent MATLAB from getting overwhelmed by too many
    % draw events, making the simulation potentially more responsive. The actual
    % limiting is controlled by how often this update function is called by the main loop.
    drawnow limitrate;

end % END FUNCTION realtime_look_depression_update


% =========================================================================
% LOCAL HELPER FUNCTIONS (for init function)
% =========================================================================

% --- Apply Theme Defaults Helper ---
function theme_out = apply_theme_defaults_rt_local(theme_in)
    % Ensures essential theme fields exist for this plot, applying defaults if missing.
    theme_out = theme_in;
    % Define fields specifically needed by this init function and provide reasonable defaults
    defaults = struct(...
        'bgColor',       [1 1 1], ... % Figure background
        'axesColor',     [1 1 1], ... % Axes background
        'axesTickColor', [0 0 0], ... % Axes lines, ticks, labels
        'gridColor',     [0.8 0.8 0.8], ... % Grid lines
        'textColor',     [0 0 0], ... % Text value color
        'labelColor',    [0 0 0], ... % Text label color
        'titleColor',    [0 0 0], ... % Plot title color
        'fontSize',      10, ...
        'fontName',      'Helvetica' ...
    );
    fields_def = fieldnames(defaults);
    for i = 1:length(fields_def)
        if ~isfield(theme_out, fields_def{i})
            theme_out.(fields_def{i}) = defaults.(fields_def{i});
        end
    end
end

% --- Helper to Apply Figure Style ---
function set_figure_style_helper_rt(fig_handle, theme, is_cyberpunk)
    % Applies background color based on theme.
    if is_cyberpunk
        set(fig_handle, 'Color', theme.bgColor);
    else
        set(fig_handle, 'Color', [0.94 0.94 0.94]); % MATLAB default grey
    end
end

% --- Helper to Apply Axes Style ---
function set_axes_style_helper_rt(ax_handle, theme, is_cyberpunk)
    % Applies colors, fonts, grid settings to axes based on theme.
    set(ax_handle, 'FontSize', theme.fontSize, 'FontName', theme.fontName);
    if is_cyberpunk
        set(ax_handle, 'Color',     theme.axesColor, ...         % Axes background
                       'XColor',    theme.axesTickColor, ...     % X-axis line, ticks, label color
                       'YColor',    theme.axesTickColor, ...     % Y-axis line, ticks, label color
                       'ZColor',    theme.axesTickColor, ...     % Z-axis line, ticks, label color (for 3D)
                       'GridColor', theme.gridColor, ...        % Major grid line color
                       'GridAlpha', 0.3);                     % Grid line transparency
                       % Optional: Minor grid styling
                       % 'MinorGridColor', theme.gridColor*0.8, 'MinorGridAlpha', 0.2);
    else % Standard MATLAB-like style
        set(ax_handle, 'Color',     'w', ...                    % White background
                       'XColor',    'k', 'YColor', 'k', 'ZColor', 'k', ... % Black axes lines/ticks/labels
                       'GridColor', [0.15 0.15 0.15], ...    % Dark grey grid
                       'GridAlpha', 0.15);
                       % 'MinorGridColor', [0.1 0.1 0.1], 'MinorGridAlpha', 0.1);
    end
end