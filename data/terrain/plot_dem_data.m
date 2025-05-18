% =========================================================================
% PLOT DEM DATA SCRIPT with Airspace Overlay
% =========================================================================

% --- Script Setup ---
clear;
clc;
close all;
fprintf('--- DEM + Airspace Visualization Script ---\n');

% --- USER CONFIGURATION ---

dem_filepath = fullfile('merged_dem.tif'); % Path to your DEM GeoTIFF
airspace_matfile = fullfile('AirspaceShape.mat'); % Path to the airspace .mat

plot_title = 'DEM + Airspace Boundaries';
z_unit = 'm';
max_points_to_plot = 1e6;
colormap_style = 'parula';
add_colorbar = true;
view_az = -35;
view_el = 30;

% --- 1. Load DEM ---
if ~isfile(dem_filepath)
    error('DEM file not found: %s', dem_filepath);
end

fprintf('Loading DEM file: %s ...\n', dem_filepath);
[Z, R] = readgeoraster(dem_filepath);
fprintf('  DEM loaded. Size: %d x %d\n', size(Z));

if ~isfloat(Z)
    Z = double(Z);
end

nodata_mask = Z < -10000;
if any(nodata_mask(:))
    Z(nodata_mask) = NaN;
    fprintf('  Replaced %d NoData values with NaN.\n', sum(nodata_mask(:)));
end

% --- 2. Geographic Grid ---
fprintf('Generating coordinate grid...\n');
[LAT, LON] = R.geographicGrid();

% --- 3. Downsampling ---
[rows, cols] = size(Z);
num_points = rows * cols;
step = 1;
if num_points > max_points_to_plot
    step = ceil(sqrt(num_points / max_points_to_plot));
    Z_sub = Z(1:step:end, 1:step:end);
    LAT_sub = LAT(1:step:end, 1:step:end);
    LON_sub = LON(1:step:end, 1:step:end);
    fprintf('  Subsampled to %d x %d\n', size(Z_sub));
else
    Z_sub = Z;
    LAT_sub = LAT;
    LON_sub = LON;
end

% --- 4. Z-Label Unit ---
if strcmpi(z_unit, 'ft')
    Z_plot = Z_sub;
    z_label_unit = 'ft';
else
    Z_plot = Z_sub;
    z_label_unit = 'm';
end

% --- 5. Create Plot ---
fprintf('Creating plot...\n');
fig = figure('Name', plot_title, 'NumberTitle', 'off');
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
view(ax, 3);

surf(ax, LON_sub, LAT_sub, Z_plot, 'EdgeColor', 'none');
colormap(ax, colormap_style);
if add_colorbar
    cb = colorbar(ax);
    ylabel(cb, sprintf('Elevation (%s)', z_label_unit));
end

title(ax, plot_title);
xlabel(ax, 'Longitude (deg)');
ylabel(ax, 'Latitude (deg)');
zlabel(ax, sprintf('Elevation MSL (%s)', z_label_unit));
axis(ax, 'tight');
axis(ax, 'normal');
view(ax, view_az, view_el);
rotate3d(ax, 'on');

% --- 6. Load and Plot Airspace Data ---
fprintf('Overlaying airspace boundaries...\n');
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
    elev = max(Z_plot(:), [], 'omitnan') + 100;

    % Plot airspace outline
    plot3(lon(valid_idx), lat(valid_idx), elev * ones(size(lat(valid_idx))), ...
          'r-', 'LineWidth', 1.5);

    % Optional: label at center
    lat_c = mean(lat(valid_idx), 'omitnan');
    lon_c = mean(lon(valid_idx), 'omitnan');
    text(lon_c, lat_c, elev + 50, name, ...
         'Color', 'red', 'FontSize', 9, 'HorizontalAlignment', 'center');
end

% load('AirspaceShape.mat'); % Loads shapeData
% 
% for i = 1:numel(shapeData)
%     entry = shapeData{i};
% 
%     % Extract name
%     try
%         name = string(entry.Name);
%     catch
%         name = "Unnamed";
%     end
% 
%     % Extract lat/lon
%     lat = entry.Latitudes;
%     lon = entry.Longitudes;
%     if iscell(lat), lat = lat{1}; end
%     if iscell(lon), lon = lon{1}; end
%     if isempty(lat) || isempty(lon), continue; end
% 
%     % Remove NaNs
%     valid_idx = ~isnan(lat) & ~isnan(lon);
%     lat = lat(valid_idx);
%     lon = lon(valid_idx);
%     if numel(lat) < 3, continue; end
% 
%     % --- Densify Boundary ---
%     % Interpolate between each pair of points to create a smoother curve
%     densified_lat = [];
%     densified_lon = [];
%     for k = 1:(length(lat)-1)
%         n_interp = 20;  % Number of points between each pair
%         lat_seg = linspace(lat(k), lat(k+1), n_interp);
%         lon_seg = linspace(lon(k), lon(k+1), n_interp);
%         densified_lat = [densified_lat, lat_seg(1:end-1)];
%         densified_lon = [densified_lon, lon_seg(1:end-1)];
%     end
%     % Add final point
%     densified_lat(end+1) = lat(end);
%     densified_lon(end+1) = lon(end);
% 
%     % --- Interpolate Terrain Elevation at Densified Points ---
%     elev = interp2(LON_sub, LAT_sub, Z_plot, densified_lon, densified_lat, 'linear', NaN);
% 
%     % Remove NaNs
%     valid_elev = ~isnan(elev);
%     if sum(valid_elev) < 3, continue; end
% 
%     % --- Plot Boundary Hugging Terrain ---
%     plot3(densified_lon(valid_elev), densified_lat(valid_elev), elev(valid_elev), ...
%           'r-', 'LineWidth', 1.5);
% 
%     % Label at center of path
%     lat_c = mean(densified_lat(valid_elev), 'omitnan');
%     lon_c = mean(densified_lon(valid_elev), 'omitnan');
%     elev_c = mean(elev(valid_elev), 'omitnan');
%     text(lon_c, lat_c, elev_c + 50, name, ...
%          'Color', 'red', 'FontSize', 9, 'HorizontalAlignment', 'center');
% end

% --- Fix axis scaling to match real-world distance ---
lat_avg = mean(LAT_sub(:), 'omitnan');
meters_per_deg_lat = 111320;
meters_per_deg_lon = 111320 * cosd(lat_avg);

z_scale_factor = 30;  % <- 1 = true vertical scale, >1 exaggerates terrain

% Set axis aspect ratio so distances scale evenly
daspect(ax, [1/meters_per_deg_lon, 1/meters_per_deg_lat, 1/z_scale_factor]);  % X:Y:Z scale


fprintf('--- Visualization Complete ---\n');
fprintf('Use mouse to rotate/pan/zoom.\n');
