clear; clc; close all; % Start fresh

%% 1) Load configuration, Earth & terrain models
fprintf('Loading configuration...\n');
cfg   = config_settings();                    % Ensure this function correctly defines paths etc.
earth = earth_model(cfg.earth.model);         % Load your earth model
tx_cfg = cfg.rf.transmitters{1};              % Pick any transmitter entry

%% 2) Flat-Earth LOS check
fprintf('Performing Flat-Earth LOS check...\n');
cfg_flat = cfg; % Create a copy to modify for flat earth
cfg_flat.terrain.ENABLE_TERRAIN_LOS = false;
terrain_flat = terrain_model(cfg_flat.terrain, earth);

% Define Points A and B
lat1 = 34.970;  lon1 = -117.930; alt1 = 5000*0.3048;  % Point A (Altitude AMSL in meters)
lat2 = 37.283555; lon2 = -116.645833; alt2 = 2207; % Point B (Original)
%lat2 = 34.970694; lon2 = -117.931; alt2 = 832; % Point B (Closer test point, Altitude AMSL in meters)

% Add a fallback check_LOS if needed (useful for testing components separately)
if ~isfield(terrain_flat, 'check_LOS') || ~isa(terrain_flat.check_LOS, 'function_handle')
    warning('terrain_flat.check_LOS not found or not a function handle. Using fallback (always returns true).');
    terrain_flat.check_LOS = @(varargin) true;
end

fprintf('Flat-Earth LOS (expect CLEAR): ');
if terrain_flat.check_LOS(lat1, lon1, alt1, lat2, lon2, alt2, tx_cfg)
    fprintf('PASSED\n');
else
    fprintf('FAILED\n');
end

%% 3) Real-DEM LOS check & Data Loading
fprintf('Performing Real-DEM LOS check...\n');
cfg.terrain.ENABLE_TERRAIN_LOS = true;
% --- IMPORTANT: Update this path to your actual DEM file ---
cfg.terrain.DEM_FILE_PATH   = 'C:\Users\pyrom\OneDrive\Desktop\Terrain Data MASTER\merged_dem.tif';
cfg.terrain.USE_GEOTIFF     = true; % Assuming terrain_model uses this

% --- Check if DEM file exists ---
if ~exist(cfg.terrain.DEM_FILE_PATH, 'file')
    error('DEM file not found at: %s\nPlease check the path.', cfg.terrain.DEM_FILE_PATH);
end

terrain = terrain_model(cfg.terrain, earth); % Initialize terrain model with DEM enabled

% --- Perform the LOS check using the terrain object method ---
fprintf('Real-DEM LOS check result: ');
try
    % Add a fallback check_LOS if needed
    if ~isfield(terrain, 'check_LOS') || ~isa(terrain.check_LOS, 'function_handle')
         warning('terrain.check_LOS not found or not a function handle. Cannot perform Real-DEM check.');
         clearLOS = NaN; % Indicate check couldn't be performed
         h_obs = NaN; d1_obs = NaN; d2_obs = NaN;
         fprintf('SKIPPED (terrain.check_LOS unavailable)\n');
    else
        [clearLOS, h_obs, d1_obs, d2_obs] = terrain.check_LOS( ...
            lat1, lon1, alt1, lat2, lon2, alt2, tx_cfg);
        if isnan(clearLOS) % Check if the function indicated an error/failure
             fprintf('FAILED (check_LOS returned NaN)\n');
        elseif clearLOS
            fprintf('CLEAR (no obstruction)\n');
        else
            fprintf('BLOCKED -> h_obstruction=%.1f m, d1=%.1f km, d2=%.1f km\n', ...
                    h_obs, d1_obs/1000, d2_obs/1000); % Display distances in km
        end
    end
catch ME
    fprintf('ERROR during terrain.check_LOS: %s\n', ME.message);
    % Optionally rethrow(ME) if you want the script to stop on error
    clearLOS = NaN; % Indicate failure
end

%% 4) Load DEM data for Visualization using readgeoraster
fprintf('Loading DEM for visualization...\n');
demFile = cfg.terrain.DEM_FILE_PATH;
try
    [Z, R] = readgeoraster(demFile, 'OutputType','double');
catch ME_read
    error('Failed to read GeoTIFF: %s\nCheck file path and format.', ME_read.message);
end

% --- Build Full Lat/Lon Grids for Interpolation ---
% Determine geographic limits from the Raster Reference Object R
if isprop(R,'LatitudeLimits') && isprop(R,'LongitudeLimits')
    latlim = R.LatitudeLimits;
    lonlim = R.LongitudeLimits;
elseif isprop(R,'YWorldLimits') && isprop(R,'XWorldLimits')
    latlim = R.YWorldLimits;
    lonlim = R.XWorldLimits;
else
    error('Cannot determine geographic limits from referencing object of class %s.', class(R));
end
nRows       = size(Z,1);
nCols       = size(Z,2);
% Create vectors representing the *center* of each cell for meshgrid
% Create vectors representing the latitude/longitude of each grid posting (row/column)
% This is the correct approach for a GeographicPostingsReference object 'R'
latVec      = linspace(latlim(1), latlim(2), nRows);
lonVec      = linspace(lonlim(1), lonlim(2), nCols);
[LonGrid, LatGrid] = meshgrid(lonVec, latVec);

% --- Interpolate ground elevation at points A and B ---
fprintf('Interpolating ground elevation at A and B...\n');
try
    z1 = interp2(LonGrid, LatGrid, Z, lon1, lat1, 'linear', NaN); % Ground elev at A
    z2 = interp2(LonGrid, LatGrid, Z, lon2, lat2, 'linear', NaN); % Ground elev at B
catch ME_interp
     error('Interpolation failed. Check if points A/B (%.4f, %.4f and %.4f, %.4f) are within DEM bounds [%.4f %.4f], [%.4f %.4f].\nError: %s', ...
         lat1, lon1, lat2, lon2, latlim(1), latlim(2), lonlim(1), lonlim(2), ME_interp.message);
end

h1 = alt1 - z1;   % Observer height above ground (AGL) at A
h2 = alt2 - z2;   % Target height above ground (AGL) at B

% --- Pre-los2 Checks ---
skip_los_processing = false; % Flag to control downstream execution

% 1) Check for NaN Heights
if isnan(h1) || isnan(h2)
    warning('Input height h1 (%.1f) or h2 (%.1f) is NaN (potentially due to interp2 failure for points outside DEM). Skipping los2 calculation and plotting.', h1, h2);
    skip_los_processing = true;
else
    % Ensure heights are non-negative only if they are not NaN
    h1(h1 < 0) = 0;
    h2(h2 < 0) = 0;
end

% 2) Check for identical points (optional but recommended)
if ~skip_los_processing && abs(lat1-lat2) < 1e-9 && abs(lon1-lon2) < 1e-9
     warning('Observer and target points have identical Lat/Lon. LOS calculation is trivial. Skipping detailed los2 profile.');
     % You could perform a simple vertical check here if needed:
     % vis = (alt1 > z2) && (alt2 > z1); % Simplistic check ignoring terrain between
     skip_los_processing = false;
end

%% 5) Calculate Visibility Profile using los2 (Conditional)
vis = NaN; visprof = []; dist = []; hLOS = []; lattrk = []; lontrk = []; % Initialize outputs

if skip_los_processing
    fprintf('Skipping LOS profile calculation.\n');
else
    fprintf('Calculating LOS profile using los2...\n');
    try
        % Call los2 (using default Earth radius)
        [vis, visprof, dist, hLOS, lattrk, lontrk] = los2(Z, R, lat1, lon1, lat2, lon2, h1, h2, 'AGL', 'AGL');

        % --- Check for Output Consistency (FIX ADDED HERE) ---
        Nvisprof = numel(visprof); % Number of points along the profile
        if Nvisprof > 0 % Only check consistency if outputs are not empty
            if numel(dist) ~= Nvisprof || numel(hLOS) ~= Nvisprof || ...
               numel(lattrk) ~= Nvisprof || numel(lontrk) ~= Nvisprof
                warning('Inconsistent array sizes returned by los2 (visprof=%d, dist=%d, hLOS=%d, lattrk=%d, lontrk=%d). Skipping down-sampling and plotting.', ...
                        Nvisprof, numel(dist), numel(hLOS), numel(lattrk), numel(lontrk));
                skip_los_processing = true; % Skip subsequent processing
            end
        elseif Nvisprof == 0 && ~(abs(lat1-lat2) < 1e-9 && abs(lon1-lon2) < 1e-9)
             warning('los2 returned an empty path profile unexpectedly.');
             skip_los_processing = true; % Skip subsequent processing
        end
        % --- End Consistency Check ---

    catch ME_los2
        fprintf('ERROR during los2 calculation: %s\n', ME_los2.message);
        skip_los_processing = true; % Skip subsequent processing on error
    end
end

%% 5.5) Down-sampling (Conditional)
% Initialize downsampled variables
lontrk_ds = []; lattrk_ds = []; hLOS_ds = []; vis_ds = []; dist_ds = [];

if skip_los_processing
    fprintf('Skipping down-sampling.\n');
else
    fprintf('Down-sampling LOS results...\n');
    Nfull = numel(visprof); % Re-get Nfull based on potentially validated visprof

    if Nfull == 0
        warning('LOS path profile contains zero points. Cannot downsample.');
    elseif Nfull == 1
        % If only one point, no need for linspace, just copy the point
        idx_ds = 1;
        fprintf('LOS path profile contains only one point.\n');
        lontrk_ds  = lontrk; % Already single element
        lattrk_ds  = lattrk;
        hLOS_ds    = hLOS;
        vis_ds     = visprof; % Should be visprof(1)
        dist_ds    = dist;    % Should be dist(1)
    else
        % Proceed with downsampling only if Nfull > 1
        Nplot   = min(Nfull, 500); % Limit plot points
        idx_ds  = round(linspace(1, Nfull, Nplot)); % indices to keep

        % --- Perform Downsampling ---
        % This section is now safe because we checked for consistency and Nfull > 0
        lontrk_ds  = lontrk(idx_ds);
        lattrk_ds  = lattrk(idx_ds);
        hLOS_ds    = hLOS(idx_ds);
        vis_ds     = visprof(idx_ds);
        dist_ds    = dist(idx_ds); % THE CRITICAL LINE - NOW SAFE
    end
end

%% 6) Plotting (Conditional)
if skip_los_processing || isempty(lontrk_ds) % Skip if issues occurred or path is empty
    fprintf('Skipping plotting due to input issues, los2 error, or empty path.\n');
else
    fprintf('Generating 3D plot...\n');
    % --- Apply Plot Theme (Optional) ---
    % ... (theme code) ...

    % --- Create Figure and Axes ---
    figure;
    ax = axes();
    hold(ax, 'on');

    % --- Plot Terrain Surface Subset ---
    % ... (terrain plotting code using LonPlot, LatPlot, ZPlot - unchanged) ...
     fprintf('Plotting terrain subset...\n'); % Status
     % Determine the lat/lon extents needed for plotting (add a small buffer)
     buffer_deg = 0.01; % Adjust buffer size as needed
     lon_plot_range = [min(lontrk_ds) - buffer_deg, max(lontrk_ds) + buffer_deg];
     lat_plot_range = [min(lattrk_ds) - buffer_deg, max(lattrk_ds) + buffer_deg];
     % Find indices ... (rest of terrain plotting code) ...

    % --- Plot LOS Path ---
    fprintf('Plotting LOS path...\n'); % Status
    if ~isempty(vis_ds) % Check if there are points to plot
        vis_ds_logical = logical(vis_ds); % Ensure logical for indexing
        % Plot visible segments
        % plot3(ax, lontrk_ds(vis_ds_logical), lattrk_ds(vis_ds_logical), hLOS_ds(vis_ds_logical), ...
        %       'Color', 'g', 'LineWidth', 2, 'DisplayName', 'Visible LOS');
        % % Plot blocked segments
        % plot3(ax, lontrk_ds(~vis_ds_logical), lattrk_ds(~vis_ds_logical), hLOS_ds(~vis_ds_logical), ...
        %       'Color', 'r', 'LineWidth', 2, 'LineStyle', '--', 'DisplayName', 'Blocked LOS');

        % Marker size in points²
markerSize = 36;  

% Visible LOS points (green filled circles)
scatter3(ax, ...
    lontrk_ds(vis_ds), ...
    lattrk_ds(vis_ds), ...
    hLOS_ds(vis_ds), ...
    markerSize, ...           % marker area
    'g', ...                  % color
    'filled', ...             % filled markers
    'DisplayName','Visible LOS');

% Blocked LOS points (red open diamonds)
scatter3(ax, ...
    lontrk_ds(~vis_ds), ...
    lattrk_ds(~vis_ds), ...
    hLOS_ds(~vis_ds), ...
    markerSize, ...
    'r', ...
    'd', ...                  % diamond marker
    'DisplayName','Blocked LOS');
    else
         disp('No LOS path segments to plot.');
    end

    % --- Mark Observer (A) & Target (B) ---
    fprintf('Plotting observer and target markers...\n'); % Status
    plot3(ax, lon1, lat1, alt1, 'ko', 'MarkerFaceColor', 'c', 'MarkerSize', 10, 'DisplayName', 'Observer A');
    plot3(ax, lon2, lat2, alt2, 'ks', 'MarkerFaceColor', 'm', 'MarkerSize', 10, 'DisplayName', 'Target B');

    % --- Beautify Plot ---
    % ... (labels, title, grid, view, legend - unchanged) ...
    hold(ax, 'off');
    fprintf('Plotting complete.\n');

    % --- Optional: Plot Profile View (2D) ---
    % ... (profile plot code - needs similar check for skip_los_processing or empty data) ...
    if ~skip_los_processing && ~isempty(dist) % Check if distance data exists for profile
        figure;
        % ... (rest of profile plot code) ...
    end

end % End conditional plotting