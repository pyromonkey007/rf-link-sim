% =========================================================================
% TEST SCRIPT: Plot Terrain, Receivers, & Path with Earth Curvature (ECEF)
%
% Description:
%   Loads a downsampled DEM, converts coordinates to ECEF, and plots:
%     1. The terrain surface using 'surf', showing Earth's curvature.
%     2. Ground station receiver locations.
%     3. A representative aircraft flight path based on waypoints.
%   Does NOT require Mapping Toolbox plotting functions (uses standard surf/plot3).
%
% Requirements:
%   - MATLAB (Base)
%   - Mapping Toolbox (likely needed for 'readgeoraster')
%   - A valid DEM file (e.g., GeoTIFF) at the specified path.
% =========================================================================
clear; clc; close all;

fprintf('Starting Terrain Curvature Test Script (ECEF Method - Downsampled)...\n');

% --- Configuration ---
demFilePath = 'C:\Users\pyrom\OneDrive\Desktop\Terrain Data MASTER\merged_dem.tif'; % DEM Path
downsample_factor = 50; % Higher = faster, less detail
aircraft_alt_ft = 18000; % Constant altitude for test flight path

% --- Receiver Locations (from config_settings.m) ---
receivers = struct(...
    'GS1', struct('lat_deg', 34.970694, 'lon_deg', -117.931, 'alt_m_msl', 832), ...
    'GS2', struct('lat_deg', 36.0765,   'lon_deg', -117.475666, 'alt_m_msl', 2480), ...
    'GS3', struct('lat_deg', 37.283555, 'lon_deg', -116.645833, 'alt_m_msl', 2207) ...
);
rx_names = fieldnames(receivers);

% --- Waypoint Locations (from config_settings.m) ---
% Using only Lat/Lon, altitude is fixed for this test path
waypoints_ll = [ ...
    % Lat      Lon
    35.64,   -117.35;    % WP1
    36.5739, -117.134;   % WP2
    37.6556, -115.398    % WP3
];
num_waypoints = size(waypoints_ll, 1);
aircraft_alt_m = aircraft_alt_ft * 0.3048; % Convert ft to meters

% --- WGS84 Ellipsoid Parameters ---
a = 6378137.0;         % Semi-major axis (meters)
f = 1/298.257223563; % Flattening
b = a * (1 - f);     % Semi-minor axis
e_sq = 1 - (b^2 / a^2); % Eccentricity squared

% --- Function Handle for LLH to ECEF Conversion ---
% (Includes the logic directly in the script for simplicity)
LLH_to_ECEF = @(lat_deg, lon_deg, alt_m) ...
    deal( ...
        (a ./ sqrt(1 - e_sq * sind(lat_deg).^2) + alt_m) .* cosd(lat_deg) .* cosd(lon_deg), ... % X
        (a ./ sqrt(1 - e_sq * sind(lat_deg).^2) + alt_m) .* cosd(lat_deg) .* sind(lon_deg), ... % Y
        ((a * (1 - e_sq)) ./ sqrt(1 - e_sq * sind(lat_deg).^2) + alt_m) .* sind(lat_deg) ... % Z
    );

% --- Check for readgeoraster ---
hasReadGeoRaster = exist('readgeoraster', 'file') == 2;
if ~hasReadGeoRaster
    warning('Function "readgeoraster" not found. Trying base MATLAB "geotiffread"...');
    hasGeoTiffRead = exist('geotiffread', 'file') == 2;
    if ~hasGeoTiffRead
         error('Neither "readgeoraster" nor "geotiffread" found. Cannot load GeoTIFF data without appropriate toolboxes.');
    end
end

% --- Load DEM Data ---
fprintf('Attempting to load DEM file: %s\n', demFilePath);
try
    if hasReadGeoRaster
        [Z_full, R] = readgeoraster(demFilePath);
        fprintf('DEM data loaded using readgeoraster.\n');
    else
        [Z_full, R_map] = geotiffread(demFilePath); R = R_map;
        fprintf('DEM data loaded using geotiffread.\n');
        if ~isprop(R,'LatitudeLimits') || ~isprop(R,'LongitudeLimits') || ~isprop(R,'RasterSize'), error('Could not determine geographic referencing from geotiffread output.'); end
    end
    info = geotiffinfo(demFilePath);
    if isfield(info, 'MissingDataIndicator') && ~isnan(info.MissingDataIndicator)
        Z_full(Z_full == info.MissingDataIndicator) = NaN;
    else
        minElev = min(Z_full(:)); if minElev < -1000, Z_full(Z_full <= minElev) = NaN; end
    end
    Z_full = double(Z_full);
    if all(isnan(Z_full(:))), error('Failed to load valid elevation data.'); end
    fprintf('Full DEM data size: %d x %d\n', size(Z_full, 1), size(Z_full, 2));
catch ME_load
    fprintf('ERROR loading DEM file: %s\n', ME_load.message); return;
end

% --- Downsample DEM Data ---
fprintf('Downsampling DEM data with factor %d...\n', downsample_factor);
Z = Z_full(1:downsample_factor:end, 1:downsample_factor:end);
[numRows, numCols] = size(Z);
fprintf('Downsampled DEM data size: %d x %d\n', numRows, numCols);

% --- Create Geographic Grid (for Downsampled DEM) ---
fprintf('Creating downsampled geographic coordinate grid...\n');
try
    latlim = R.LatitudeLimits; lonlim = R.LongitudeLimits; fullRasterSize = R.RasterSize;
    latVec_full = linspace(latlim(1), latlim(2), fullRasterSize(1));
    lonVec_full = linspace(lonlim(1), lonlim(2), fullRasterSize(2));
    latVec = latVec_full(1:downsample_factor:end);
    lonVec = lonVec_full(1:downsample_factor:end);
    [LonGrid, LatGrid] = meshgrid(lonVec, latVec);
    if size(LonGrid, 1) ~= numRows || size(LonGrid, 2) ~= numCols, error('Downsampled Lat/Lon grid does not match downsampled DEM dimensions.'); end
    fprintf('Downsampled geographic grid created.\n');
catch ME_grid
     fprintf('ERROR creating geographic grid: %s\n', ME_grid.message); return;
end

% --- Convert Downsampled DEM Grid to ECEF ---
fprintf('Converting %d DEM points to ECEF...\n', numel(Z));
[X_ecef_grid, Y_ecef_grid, Z_ecef_grid] = LLH_to_ECEF(LatGrid, LonGrid, Z);
nan_mask = isnan(Z);
X_ecef_grid(nan_mask) = NaN; Y_ecef_grid(nan_mask) = NaN; Z_ecef_grid(nan_mask) = NaN;
fprintf('DEM ECEF conversion complete.\n');

% --- Convert Receiver Locations to ECEF ---
fprintf('Converting receiver locations to ECEF...\n');
Rx_ECEF_X = zeros(length(rx_names), 1);
Rx_ECEF_Y = zeros(length(rx_names), 1);
Rx_ECEF_Z = zeros(length(rx_names), 1);
for i = 1:length(rx_names)
    rx_name = rx_names{i};
    [Rx_ECEF_X(i), Rx_ECEF_Y(i), Rx_ECEF_Z(i)] = LLH_to_ECEF(...
        receivers.(rx_name).lat_deg, ...
        receivers.(rx_name).lon_deg, ...
        receivers.(rx_name).alt_m_msl);
end
fprintf('Receiver ECEF conversion complete.\n');

% --- Convert Waypoint Locations to ECEF ---
fprintf('Converting waypoint locations to ECEF (Alt = %.1f m)...\n', aircraft_alt_m);
Wp_ECEF_X = zeros(num_waypoints, 1);
Wp_ECEF_Y = zeros(num_waypoints, 1);
Wp_ECEF_Z = zeros(num_waypoints, 1);
for i = 1:num_waypoints
    [Wp_ECEF_X(i), Wp_ECEF_Y(i), Wp_ECEF_Z(i)] = LLH_to_ECEF(...
        waypoints_ll(i, 1), ... % Lat
        waypoints_ll(i, 2), ... % Lon
        aircraft_alt_m);        % Constant Altitude
end
fprintf('Waypoint ECEF conversion complete.\n');


% --- Create Plot ---
fprintf('Creating ECEF surface plot...\n');
figure('Name', 'Terrain Curvature Test (ECEF - Downsampled)', 'Position', [100, 100, 900, 700]);
ax = axes;

% --- Plot Terrain Surface ---
surf(ax, X_ecef_grid, Y_ecef_grid, Z_ecef_grid, Z, 'EdgeColor', 'none'); % Color surface by elevation Z
hold(ax, 'on'); % Hold axes for adding receivers and path

% --- Plot Receivers ---
scatter3(ax, Rx_ECEF_X, Rx_ECEF_Y, Rx_ECEF_Z, ...
         100, 'm', '^', 'filled', 'MarkerEdgeColor', 'k', ...
         'DisplayName', 'Receivers');
% Add labels slightly offset
text(ax, Rx_ECEF_X + 100, Rx_ECEF_Y + 100, Rx_ECEF_Z, rx_names, ...
     'Color', 'm', 'FontWeight', 'bold');

% --- Plot Flight Path ---
plot3(ax, Wp_ECEF_X, Wp_ECEF_Y, Wp_ECEF_Z, ...
      'r-o', 'LineWidth', 2, 'MarkerSize', 6, ...
      'MarkerFaceColor', 'r', 'DisplayName', sprintf('Flight Path (%.0f ft)', aircraft_alt_ft));

hold(ax, 'off');

% --- Customize Appearance ---
title(ax, sprintf('Terrain, Receivers & Path in ECEF (DEM Downsampled by %d)', downsample_factor));
xlabel(ax, 'ECEF X (m)');
ylabel(ax, 'ECEF Y (m)');
zlabel(ax, 'ECEF Z (m)');
axis(ax, 'equal'); % Crucial for true shape
rotate3d(ax, 'on');
grid(ax, 'on');
view(ax, 3);
colormap(ax, parula); % Use non-conflicting colormap

% *** FIX for Colorbar Label ***
cb = colorbar; % Get colorbar handle
cb.Label.String = 'Elevation (m)'; % Set label using handle

legend(ax, 'Location', 'best');

% Optional: Center the view roughly on the data
try
    % Calculate center based on path and terrain extent
    all_X = [X_ecef_grid(:); Wp_ECEF_X(:); Rx_ECEF_X(:)];
    all_Y = [Y_ecef_grid(:); Wp_ECEF_Y(:); Rx_ECEF_Y(:)];
    all_Z = [Z_ecef_grid(:); Wp_ECEF_Z(:); Rx_ECEF_Z(:)];
    meanX = mean(all_X,'omitnan'); meanY = mean(all_Y,'omitnan'); meanZ = mean(all_Z,'omitnan');
    if all(isfinite([meanX, meanY, meanZ]))
        camtarget(ax, [meanX, meanY, meanZ]);
        camva(ax, max(camva(ax)/2, 5)); % Zoom out
    end
catch
end

fprintf('Plot created successfully.\n');
% --- End of Script ---
fprintf('Test script finished.\n');