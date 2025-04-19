filename = 'merged_dem.tif';
[elevationData, R] = readgeoraster(filename);
% Define the latitude and longitude coordinates
latitude = 36.5739;  % Example latitude 34.9141151642
longitude = -117.134;  % Example longitude -117.9169007855
% should be 708.08 meters
% actual return was 263 meters....
% Convert latitude and longitude to row and column indices
[row, col] = geographicToIntrinsic(R, latitude, longitude);
% Extract the elevation value
elevation = elevationData(round(row), round(col));
% Display the elevation value
fprintf('The elevation at latitude %.4f and longitude %.4f is %.2f meters.\n', latitude, longitude, elevation);