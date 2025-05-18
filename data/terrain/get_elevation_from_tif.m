filename = 'C:\Users\pyrom\OneDrive\Desktop\Terrain Data MASTER\merged_dem.tif';
[elevationData, R] = readgeoraster(filename);
% Define the latitude and longitude coordinates
latitude = 34.970694;  % Example latitude 34.9141151642
longitude = -117.931;  % Example longitude -117.9169007855
% should be 708.08 meters
% actual return was 263 meters....
% Convert latitude and longitude to row and column indices
[row, col] = geographicToIntrinsic(R, latitude, longitude);
% Extract the elevation value
elevation = elevationData(round(row), round(col));
% Display the elevation value
fprintf('The elevation at latitude %.4f and longitude %.4f is %.2f meters.\n', latitude, longitude, elevation);


% --- Load the DEM and its referencing object ---
[Z, R] = readgeoraster(config.terrain.DEM_FILE_PATH);

% 2) Convert (lat,lon) → discrete grid indices (row, col)
%    geographicToDiscrete rounds for you
[row, col] = geographicToDiscrete(R, latitude, longitude);

% 3) Clip to valid range
row = max(1, min(row, size(Z,1)));
col = max(1, min(col, size(Z,2)));

% 4) Read the raw DEM elevation
elevation = Z(row, col);
fprintf('Raw DEM at (%f, %f) = %.1f m\n', 34.9707, -117.9310, elevation);

% 5) Compare with your interpolated lookup
terrain = terrain_model(config.terrain);
interpValue = terrain.get_altitude(latitude, longitude);
fprintf('Interpolated DEM elevation = %.1f m\n', interpValue);