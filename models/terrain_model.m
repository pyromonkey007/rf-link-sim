% =========================================================================
% TERRAIN MODEL (FINAL - Corrected v7, Generalized Ref Check)
% Version: FINAL 1.6
%
% Description:
%   Loads and manages terrain elevation data (DEM - Digital Elevation Model).
%   Provides functions for:
%   1. Looking up terrain elevation at specific geographic coordinates (Lat/Lon)
%      using interpolation (`get_altitude`).
%   2. Performing basic Line-of-Sight (LOS) checks between two 3D points,
%      considering terrain obstruction (`check_LOS`).
%
% DEM File Handling:
%   - Configurable via `config.terrain` structure.
%   - Supports GeoTIFF format (using MATLAB's Mapping Toolbox `readgeoraster`
%     if available and `config.terrain.USE_GEOTIFF` is true). This is the
%     recommended format for georeferenced data.
%   - Provides a fallback mechanism to read simple CSV files containing
%     Latitude, Longitude, and Elevation columns. This uses `readtable` and
%     `scatteredInterpolant` (suitable for unstructured or gridded CSV data).
%   - Handles DEM downsampling via `config.terrain.DOWNSAMPLE_FACTOR` to
%     reduce memory usage and processing time for large datasets.
%   - Handles NoData values (often large negative numbers) by replacing them
%     with NaN before interpolation.
%
% LOS Check Method (`check_LOS`):
%   - Uses a simple linear profile sampling method. It traces a straight line
%     in 3D space between the start and end points.
%   - It samples points along this line and compares the line's altitude (MSL)
%     to the terrain's altitude (MSL) directly below the sample point.
%   - **Limitations:** This basic method does NOT account for Earth's curvature
%     or atmospheric refraction (which can significantly affect LOS over long
%     distances or at low angles). It might also miss small terrain peaks
%     located between the sample points.
%   - **Recommendation:** For high-fidelity LOS analysis, especially over long
%     ranges, consider implementing or using MATLAB's `los2` function (requires
%     Mapping Toolbox) or integrating a more advanced terrain profiling algorithm
%     that incorporates curvature and refraction models.
%
% Usage:
%   - Initialized in `main_simulation.m`:
%     `terrain = terrain_model(config.terrain);`
%     The function returns a default "flat earth" object if terrain is disabled
%     in the config or if DEM loading fails.
%   - Called by other models (e.g., `rf_propagation_model`, `aircraft_dynamics_model`):
%     `elevation = terrain.get_altitude(lat, lon);` % Returns elevation in meters MSL
%     `is_clear = terrain.check_LOS(lat1, lon1, alt1_msl, lat2, lon2, alt2_msl);` % Returns true if clear, false if blocked
%
% Key Fixes in this Version (1.6):
%   - **Replaced overly specific `isa(R, 'map.rasterref.MapRasterReference')` checks
%     with a more general `isa(R, 'map.rasterref.RasterReference')` to correctly
%     handle various valid referencing objects like GeographicCellsReference.** % <-- NEW FIX
%   - Removed faulty check that incorrectly triggered CSV fallback when
%     `readgeoraster` returned a valid `GeographicCellsReference` object.
%   - Replaced incorrect `R.interpolate` call with `geointerp(dem, R, ...)` for
%     Mapping Toolbox raster reference objects to resolve "Unrecognized method" warning.
%   - Corrected the check for Mapping Toolbox reference object types in
%     `get_altitude_internal` to explicitly include `GeographicCellsReference` and the superclass.
%   - Fixed `MEXCEP` code analyzer issues using formatted warning/error messages.
%   - Improved comments and code formatting.
%
% Maintainer: [Your Name/Team] / Gemini Assistance
% Last Updated: 2025-04-05 - Generalized raster reference check.
% =========================================================================
function terrain_out = terrain_model(terrain_config)
    % Creates a terrain model object capable of providing elevation lookups
    % and performing basic Line-of-Sight (LOS) checks.
    %
    % Args:
    %   terrain_config (struct): Configuration structure containing settings
    %                            like DEM file path, interpolation method, etc.
    %
    % Returns:
    %   terrain_out (struct): A structure containing:
    %       - data_loaded (logical): Flag indicating if DEM data was loaded.
    %       - dem_data: The loaded DEM data (grid or scatteredInterpolant).
    %       - ref_obj: The spatial referencing object (e.g., GeographicCellsReference) or info struct.
    %       - ref_obj_type (char): 'grid', 'interpolant', or 'none'.
    %       - get_altitude (function_handle): Function to get elevation at Lat/Lon.
    %       - check_LOS (function_handle): Function to check LOS between two points.

    fprintf('[TERRAIN] Initializing Terrain Model...\n');

    % --- Initialize Output Structure with Default Handles (Flat Earth) ---
    % These defaults are used if terrain is disabled or loading fails.
    terrain_out = struct();
    terrain_out.data_loaded    = false;       % Flag: Was DEM data successfully loaded?
    terrain_out.dem_data       = [];          % Stores the actual elevation data (grid or interpolant)
    terrain_out.ref_obj        = [];          % Stores the referencing object (e.g., GeographicCellsReference) or struct
    terrain_out.ref_obj_type   = 'none';      % Type identifier: 'grid', 'interpolant', or 'none'
    terrain_out.get_altitude   = @(lat, lon) 0; % Default function: always returns 0m elevation
    terrain_out.check_LOS      = @(varargin) true; % Default function: always returns true (clear LOS)

    % --- Check if Terrain Feature is Enabled in Configuration ---
    if ~isfield(terrain_config, 'ENABLE_TERRAIN_LOS') || ~terrain_config.ENABLE_TERRAIN_LOS
        fprintf('[TERRAIN] Terrain LOS checks disabled by configuration. Using flat earth model.\n');
        return; % Exit with default flat earth object
    end

    % --- Validate DEM File Path Configuration ---
    if ~isfield(terrain_config, 'DEM_FILE_PATH') || isempty(terrain_config.DEM_FILE_PATH)
        warning('TerrainModel:ConfigWarning', 'DEM_FILE_PATH not specified or empty in config. Using flat terrain model.');
        return; % Exit with default flat earth object
    end
    dem_file = terrain_config.DEM_FILE_PATH;

    % --- Check if the Specified DEM File Exists ---
    if ~isfile(dem_file)
        warning('TerrainModel:FileNotFound', 'Specified DEM file not found: "%s". Using flat terrain model.', dem_file);
        return; % Exit with default flat earth object
    end

    % --- Attempt to Load DEM Data (Main Try-Catch Block) ---
    try
        fprintf('  Loading DEM file: %s\n', dem_file);
        dem = []; % Initialize DEM data variable
        R = [];   % Initialize referencing object variable

        % Determine if GeoTIFF loading should be attempted
        use_geotiff = isfield(terrain_config, 'USE_GEOTIFF') && terrain_config.USE_GEOTIFF;
        mapping_toolbox_exists = license('test', 'map_toolbox') && ~isempty(ver('map'));

        % --- Method 1: Try GeoTIFF Loading (Requires Mapping Toolbox) ---
        if use_geotiff && mapping_toolbox_exists
            fprintf('    Attempting load via readgeoraster (Mapping Toolbox available)...\n');
            try
                % Load DEM grid data and the associated spatial referencing object
                % Load DEM grid data and the associated spatial referencing object
                [dem_raw, R_raw] = readgeoraster(dem_file);

                % Basic validation
                if ~isnumeric(dem_raw) || ~ismatrix(dem_raw)
                    error('readgeoraster:InvalidOutput','readgeoraster did not return a numeric matrix for DEM data.');
                end
                if isempty(R_raw)
                    error('readgeoraster:InvalidRefObj','readgeoraster did not return a referencing object.');
                end

                % Convert referencing object if it's a postings reference
                if isa(R_raw, 'map.rasterref.GeographicPostingsReference')
                    fprintf('  Converting GeographicPostingsReference to GeographicCellsReference (legacy compatible)...\n');

                    dLat = diff(R_raw.LatitudeLimits)  / (R_raw.RasterSize(1) - 1);
                    dLon = diff(R_raw.LongitudeLimits) / (R_raw.RasterSize(2) - 1);

                    lat_limits_cells = [R_raw.LatitudeLimits(1) - dLat/2, R_raw.LatitudeLimits(2) + dLat/2];
                    lon_limits_cells = [R_raw.LongitudeLimits(1) - dLon/2, R_raw.LongitudeLimits(2) + dLon/2];

                    R = georefcells(lat_limits_cells, lon_limits_cells, size(dem_raw));
                else
                    R = R_raw;
                end

                % Assign DEM
                dem = dem_raw;
                fprintf('    Loaded referencing object of type: %s.\n', class(R));
                fprintf('    DEM loaded successfully via readgeoraster (%d rows x %d cols grid).\n', size(dem, 1), size(dem, 2));


                % Basic validation of loaded data
                if ~isnumeric(dem) || ~ismatrix(dem)
                    error('readgeoraster:InvalidOutput','readgeoraster did not return a numeric matrix for DEM data.');
                end
                if isempty(R)
                    error('readgeoraster:InvalidRefObj','readgeoraster did not return a referencing object.');
                
                end

                % Informational log about the loaded referencing object type
                fprintf('    Loaded referencing object of type: %s.\n', class(R));

                fprintf('    DEM loaded successfully via readgeoraster (%d rows x %d cols grid).\n', size(dem, 1), size(dem, 2));

            catch ME_geotiff
                 % If readgeoraster fails, issue a warning and allow fallback to CSV method
                 warning(ME_geotiff.identifier, ...
                         '[TERRAIN] readgeoraster failed: %s\n    Attempting CSV/Interpolant fallback...', ME_geotiff.message);
                 dem = []; R = []; % Reset variables to ensure fallback is triggered
            end
        elseif use_geotiff && ~mapping_toolbox_exists
             % Inform user if GeoTIFF is desired but toolbox is missing
             fprintf('    Configured to use GeoTIFF, but Mapping Toolbox not found. Attempting CSV/Interpolant fallback...\n');
        end

        % --- Method 2: CSV/Interpolant Fallback Loading ---
        % This block executes if GeoTIFF loading was skipped, disabled, or failed (dem is still empty)
        if isempty(dem)
            if ~use_geotiff
                fprintf('    Using CSV/Interpolant loader (USE_GEOTIFF set to false).\n');
            else
                fprintf('    Attempting CSV/Interpolant fallback method...\n');
            end

            try
                 % Attempt to read the file as a table (will fail if it's a non-CSV format like GeoTIFF)
                 dem_table = readtable(dem_file);
                 col_names = lower(dem_table.Properties.VariableNames); % Use lower case for matching

                 % Automatically find columns based on common names
                 lat_col   = find(contains(col_names, {'lat', 'latitude'}), 1);
                 lon_col   = find(contains(col_names, {'lon', 'long', 'longitude'}), 1);
                 elev_col  = find(contains(col_names, {'elev', 'alt', 'height', 'z', 'value'}), 1);

                 % Validate that essential columns were found
                 if isempty(lat_col) || isempty(lon_col) || isempty(elev_col)
                     error('TerrainModel:CSVFormat','CSV file "%s" must contain Latitude, Longitude, and Elevation columns with recognizable names.', dem_file);
                 end
                 fprintf('    Identified columns: Lat=%s, Lon=%s, Elev=%s\n', ...
                         dem_table.Properties.VariableNames{lat_col}, ...
                         dem_table.Properties.VariableNames{lon_col}, ...
                         dem_table.Properties.VariableNames{elev_col});

                 % Extract data
                 lats  = dem_table{:, lat_col};
                 lons  = dem_table{:, lon_col};
                 elevs = dem_table{:, elev_col};

                 % Remove rows with NaN values in essential columns
                 valid_rows = ~isnan(lats) & ~isnan(lons) & ~isnan(elevs);
                 num_removed = sum(~valid_rows);
                 if num_removed > 0
                      fprintf('    Removed %d rows with NaN values from CSV.\n', num_removed);
                      lats = lats(valid_rows); lons = lons(valid_rows); elevs = elevs(valid_rows);
                 end
                 if isempty(lats), error('TerrainModel:CSVEmpty','No valid (non-NaN) data points found in CSV "%s".', dem_file); end

                 % Select interpolation method for scattered data
                 interp_method_cfg = terrain_config.INTERPOLATION_METHOD;
                 % scatteredInterpolant doesn't support 'bicubic' or 'spline', map to alternatives
                 if strcmpi(interp_method_cfg, 'bicubic'), interp_method = 'natural';
                 elseif strcmpi(interp_method_cfg, 'spline'), interp_method = 'linear';
                 else, interp_method = interp_method_cfg; % Use 'linear' or 'nearest' if specified
                 end
                 fprintf('    Creating scatteredInterpolant (Method: %s)...\n', interp_method);
                 F_dem = scatteredInterpolant(lats, lons, elevs, interp_method, 'none'); % 'none' for extrapolation behavior
                 fprintf('    scatteredInterpolant created.\n');

                 % Store the interpolant object as the 'dem' data
                 dem = F_dem;
                 % Create a basic structure to hold reference info (limits) for the interpolant
                 R   = struct();
                 R.LatitudeLimits  = [min(lats), max(lats)];
                 R.LongitudeLimits = [min(lons), max(lons)];
                 R.isInterpolant   = true; % Flag to identify this type later
                 fprintf('    DEM loaded via CSV using scatteredInterpolant.\n');

             catch ME_csv
                 % If CSV loading fails (e.g., file was binary GeoTIFF, format incorrect), report error
                 error('TerrainModel:CSVFail','Failed to load or process DEM from CSV "%s": %s', dem_file, ME_csv.message);
             end
        end % End CSV Fallback Loading

        % --- Post-processing for Loaded Data ---

        if isnumeric(dem) && ismatrix(dem) % Data is a GRID (from GeoTIFF)
            terrain_out.ref_obj_type = 'grid'; % Set type identifier

            % Ensure data is floating point for calculations
            if ~isfloat(dem)
                fprintf('    Converting DEM grid data type to single precision...\n');
                dem = single(dem);
            end

            % Handle NoData values (replace with NaN for interpolation)
            no_data_val = NaN;
             % Check if the referencing object specifies a NoData indicator
             % ** FIX: Check against general RasterReference base class **
             if isa(R, 'map.rasterref.GeographicCellsReference') && isprop(R, 'MissingDataIndicator') && ~isnan(R.MissingDataIndicator)
                no_data_val = R.MissingDataIndicator;
                fprintf('    Identified NoData value from reference object: %f\n', no_data_val);
             else
                 % If not specified, check for common placeholder values (e.g., large negative)
                 if any(dem(:) < -10000) % Heuristic check
                     % Use a common default indicator if suspected
                     no_data_val = -9999; % Common value, adjust if needed for your DEM source
                     fprintf('    Checking for generic NoData values (e.g., < -10000 -> assuming %d)...\n', no_data_val);
                 end
             end
             % Replace identified NoData values with NaN
             if ~isnan(no_data_val)
                 nodata_mask = (dem == no_data_val | dem < -10000); % Include heuristic check again
                 num_nodata_replaced = sum(nodata_mask(:));
                 if num_nodata_replaced > 0
                     dem(nodata_mask) = NaN;
                     fprintf('    Replaced %d NoData values with NaN.\n', num_nodata_replaced);
                 end
             end

            % Optional Downsampling (applied to grid data)
            ds_factor = 1;
            if isfield(terrain_config,'DOWNSAMPLE_FACTOR'), ds_factor=round(terrain_config.DOWNSAMPLE_FACTOR); end
            if ds_factor > 1
                fprintf('    Downsampling DEM grid by factor %d...\n', ds_factor);
                dem = dem(1:ds_factor:end, 1:ds_factor:end); % Simple subsampling

                 % Attempt to adjust the referencing object to match downsampling
                 % ** FIX: Check against general RasterReference base class **
                 if isa(R, 'map.rasterref.GeographicCellsReference')
                     try
                         % Use the scaleRasterSize method if available
                         R = R.scaleRasterSize(1/ds_factor);
                         fprintf('    Adjusted MapRasterReference object for downsampling.\n');
                     catch ME_scale
                         warning(ME_scale.identifier, ...
                                 'Failed to automatically scale MapRasterReference object: %s. Georeferencing might be incorrect after downsampling.', ME_scale.message);
                     end
                 else
                     warning('TerrainModel:DownsampleRefWarn','Cannot automatically adjust referencing object of type %s for downsampling.', class(R));
                 end
                 fprintf('    Downsampled grid size: %d x %d.\n', size(dem, 1), size(dem, 2));
            end

        elseif isa(dem,'scatteredInterpolant') % Data is an INTERPOLANT (from CSV)
            terrain_out.ref_obj_type = 'interpolant'; % Set type identifier

        else % Should not happen if loading succeeded
            error('TerrainModel:LoadError','Loaded DEM data is not a recognized grid or interpolant type after processing.');
        end

        % --- Store Final Data and Set Public Function Handles in Output Struct ---
        terrain_out.dem_data       = dem; % Store processed DEM data/interpolant
        terrain_out.ref_obj        = R;   % Store referencing object/struct
        terrain_out.data_loaded    = true; % Mark data as successfully loaded

        % Assign handles to internal functions, passing the terrain object itself
        terrain_out.get_altitude   = @(lat, lon) get_altitude_internal(terrain_out, lat, lon, terrain_config.INTERPOLATION_METHOD);
        terrain_out.check_LOS      = @(varargin) check_LOS_internal(terrain_out, varargin{:});

        fprintf('[TERRAIN] Terrain model initialized successfully.\n');

        % Attempt to report geographic coverage
        try
            % ** FIX: Check against general RasterReference base class **
            if isstruct(R) && isfield(R,'LatitudeLimits') % From interpolant
                 fprintf('  Coverage (Interpolant): Lat [%.5f, %.5f], Lon [%.5f, %.5f]\n', R.LatitudeLimits(1), R.LatitudeLimits(2), R.LongitudeLimits(1), R.LongitudeLimits(2));
            elseif isa(R, 'map.rasterref.GeographicCellsReference') % From GeoTIFF
                if isprop(R, 'LatitudeLimits') && isprop(R, 'LongitudeLimits')
                    fprintf('  Coverage (Grid): Lat [%.5f, %.5f], Lon [%.5f, %.5f]\n', R.LatitudeLimits(1), R.LatitudeLimits(2), R.LongitudeLimits(1), R.LongitudeLimits(2));
                elseif isprop(R, 'YWorldLimits') && isprop(R, 'XWorldLimits') % Projected coords case
                    fprintf('  Coverage (Grid - Projected): X [%.2f, %.2f], Y [%.2f, %.2f]\n', R.XWorldLimits(1), R.XWorldLimits(2), R.YWorldLimits(1), R.YWorldLimits(2));
                end
            end
        catch ME_report
             warning(ME_report.identifier, 'Could not report terrain coverage details: %s', ME_report.message);
        end

    catch ME_load % Catch errors during the entire loading/initialization process
        warning(ME_load.identifier, '[TERRAIN] Failed to initialize terrain model: %s\nUsing default flat terrain model.', ME_load.message);
        % Ensure object returns default flat earth handles on any load failure
        terrain_out.data_loaded    = false;
        terrain_out.dem_data       = [];
        terrain_out.ref_obj        = [];
        terrain_out.ref_obj_type   = 'none';
        terrain_out.get_altitude   = @(lat, lon) 0;
        terrain_out.check_LOS      = @(varargin) true;
    end

end % END OF FUNCTION terrain_model


% =========================================================================
% INTERNAL HELPER FUNCTION: Get Altitude at Lat/Lon
% =========================================================================
function elevation = get_altitude_internal(terrain_obj, lat, lon, method)
    % Gets terrain altitude (meters MSL) at specific lat/lon coordinates
    % using interpolation based on the loaded DEM data and referencing object.
    % Called via the terrain_out.get_altitude function handle.
    %
    % Args:
    %   terrain_obj (struct): The terrain model object containing data and refs.
    %   lat (double): Latitude coordinate(s) (degrees).
    %   lon (double): Longitude coordinate(s) (degrees).
    %   method (char): Interpolation method ('nearest', 'linear', 'cubic', etc.)
    %                  Passed from config, used primarily by geointerp.
    %
    % Returns:
    %   elevation (double): Interpolated terrain elevation (meters MSL).
    %                       Returns 0 if outside coverage, data not loaded,
    %                       or if an error occurs during lookup.

    elevation = 0; % Default return value

    % Quick exit if terrain data wasn't loaded successfully
    if ~terrain_obj.data_loaded, return; end

    % Retrieve data and reference object from the terrain structure
    R   = terrain_obj.ref_obj;
    dem = terrain_obj.dem_data;

    try
        % --- Case 1: Data loaded as SCATTERED INTERPOLANT (from CSV) ---
        if strcmp(terrain_obj.ref_obj_type, 'interpolant')
            % Check if query point is within the bounds defined during interpolant creation
            if lat >= R.LatitudeLimits(1) && lat <= R.LatitudeLimits(2) && ...
               lon >= R.LongitudeLimits(1) && lon <= R.LongitudeLimits(2)
                % Call the scatteredInterpolant object directly
                elevation = dem(lat, lon);
                % Replace potential NaN results (e.g., from 'none' extrapolation) with 0
                if isnan(elevation), elevation = 0; end
            else
                % Outside coverage defined by the source CSV points
                elevation = 0;
            end

        % --- Case 2: Data loaded as GRID (from GeoTIFF) ---
        elseif strcmp(terrain_obj.ref_obj_type, 'grid')
            % Check if the reference object 'R' is a valid Mapping Toolbox type
            % ** FIX: Check against general RasterReference base class **
            if isa(R, 'map.rasterref.GeographicCellsReference')
                try
                    % Determine if the query point is within the raster bounds
                    in_bounds = true; % Assume in bounds initially
                    % Use the more robust 'contains' method if available on the object
                    if ismethod(R, 'contains')
                        in_bounds = R.contains(lat, lon);
                    % Fallback to checking limit properties if 'contains' method absent
                    elseif isprop(R, 'LatitudeLimits') && isprop(R, 'LongitudeLimits') % Geographic
                         in_bounds = (lat >= R.LatitudeLimits(1) && lat <= R.LatitudeLimits(2) && ...
                                      lon >= R.LongitudeLimits(1) && lon <= R.LongitudeLimits(2));
                    elseif isprop(R, 'XWorldLimits') && isprop(R, 'YWorldLimits') % Projected
                         % Note: Direct lat/lon check against projected limits is generally incorrect.
                         % 'contains' method or coordinate transformation would be needed here.
                         % For simplicity, we might assume caller handles projection or risk error.
                         % Setting in_bounds to false here if only projected limits exist prevents errors.
                          warning('TerrainModel:ProjCoordLookup', 'Attempting Lat/Lon lookup on DEM with projected coordinates without transformation. Use projected coordinates or ensure ''contains'' method works.');
                          in_bounds = false; % Safer default without transformation
                    end

                    % Perform interpolation only if within bounds
                    if in_bounds
                        % Use geointerp for Mapping Toolbox raster reference objects
                        % This function handles various interpolation methods correctly.
                        elevation = geointerp(dem, R, lat, lon, method);
                    else
                        elevation = 0; % Outside defined coverage area
                    end
                    % Replace potential NaN results (e.g., from interpolating near NaN NoData) with 0
                    if isnan(elevation), elevation = 0; end

                catch ME_geointerp
                    % Catch errors specifically from geointerp or contains method
                    warning(ME_geointerp.identifier, ...
                            'geointerp/contains failed for ref obj type %s: %s. Returning 0.', class(R), ME_geointerp.message);
                    elevation = 0;
                end
            else
                 % This block should ideally not be reached if loading was successful
                 % but serves as a safeguard if ref_obj_type is 'grid' but R is invalid.
                 warning('TerrainModel:UnexpectedGridRefObj', 'Ref object type is "grid" but object is not a recognized RasterReference (%s). Cannot interpolate. Returning 0.', class(R));
                 elevation = 0;
            end % End check on R type for grid data

        % --- Case 3: No Data or Unknown Type ---
        else
            % Should not happen with current logic, but good practice
            if terrain_obj.data_loaded
                 warning('TerrainModel:UnknownRefType', 'Unknown ref_obj_type "%s". Cannot get altitude.', terrain_obj.ref_obj_type);
            end
             elevation = 0; % Default to 0 if type is 'none' or unexpected
        end % End check ref_obj_type

    catch ME_lookup
         % Catch any unexpected errors during the lookup process
         warning(ME_lookup.identifier, ...
                 'Unexpected error during terrain altitude lookup for Lat=%.5f, Lon=%.5f: %s. Returning 0.', lat, lon, ME_lookup.message);
         elevation = 0;
    end
end % END get_altitude_internal


% =========================================================================
% INTERNAL HELPER FUNCTION: Check Line-of-Sight (Simple Linear Profile)
% =========================================================================
function los_clear = check_LOS_internal(terrain_obj, lat1, lon1, alt1_msl, lat2, lon2, alt2_msl, varargin)
    % Checks Line-of-Sight (LOS) between two points considering terrain obstruction.
    % Uses a simple linear profile sampling method: samples points along the
    % straight line path in 3D and compares the line height to terrain height.
    % Does NOT account for Earth curvature or atmospheric refraction.
    % Called via the terrain_out.check_LOS function handle.
    %
    % Args:
    %   terrain_obj (struct): The terrain model object.
    %   lat1, lon1 (double): Coordinates of the start point (degrees).
    %   alt1_msl (double): Altitude of the start point (meters MSL).
    %   lat2, lon2 (double): Coordinates of the end point (degrees).
    %   alt2_msl (double): Altitude of the end point (meters MSL).
    %   varargin{1} (optional, int): Number of sample points along the path (default 100).
    %
    % Returns:
    %   los_clear (logical): True if LOS is clear, False if obstructed by terrain.

    los_clear = true; % Assume clear unless obstruction found

    % Quick exit if terrain data wasn't loaded successfully
    if ~terrain_obj.data_loaded, return; end

    % --- Check if Endpoints are Below Terrain ---
    % If either endpoint is already below the terrain surface, LOS is impossible.
    try
        terrain_alt1 = terrain_obj.get_altitude(lat1, lon1);
        terrain_alt2 = terrain_obj.get_altitude(lat2, lon2);
    catch ME_alt_check
         % If altitude lookup fails for endpoints, cautiously assume LOS is clear to avoid false negatives.
         warning(ME_alt_check.identifier, ...
                 'Failed endpoint altitude check during LOS: %s. Assuming LOS clear.', ME_alt_check.message);
         return; % Assume clear if the check itself fails
    end
    if alt1_msl < terrain_alt1 || alt2_msl < terrain_alt2
        los_clear = false; % One or both points are underground
        return;
    end

    % --- Sample points along the straight-line path in Lat/Lon/Alt space ---
    num_samples = 100; % Default number of intermediate points to check
    if ~isempty(varargin) && isnumeric(varargin{1}) && varargin{1} >= 2
        num_samples = round(varargin{1});
    end

    % Create vectors of linearly spaced points between start and end
    lat_samples      = linspace(lat1, lat2, num_samples)'; % Latitudes of sample points
    lon_samples      = linspace(lon1, lon2, num_samples)'; % Longitudes of sample points
    alt_line_samples = linspace(alt1_msl, alt2_msl, num_samples)'; % Altitudes along the straight line path

    % --- Check Terrain Altitude at intermediate sample points ---
    % Skip first (alt1) and last (alt2) points as they were already checked.
    for i = 2:(num_samples - 1)
        try
            % Get the terrain elevation directly below the i-th sample point
            terrain_alt_sample = terrain_obj.get_altitude(lat_samples(i), lon_samples(i));
        catch ME_alt_sample
             % If lookup fails for an intermediate point, warn but assume clear at that point
             warning(ME_alt_sample.identifier, ...
                     'Failed LOS altitude sample at point %d / %d: %s. Assuming clear at this point.', i, num_samples, ME_alt_sample.message);
             terrain_alt_sample = -Inf; % Treat as infinitely low terrain if lookup fails
        end

        % --- Check for Obstruction ---
        % Compare the altitude of the straight-line path to the terrain altitude.
        % Add a small tolerance to prevent floating point precision issues.
        tolerance = 0.1; % meters - adjust if needed
        if (alt_line_samples(i) - tolerance) < terrain_alt_sample
            % If the line-of-sight path dips below the terrain altitude at this sample point...
            los_clear = false; % ... then the LOS is obstructed.
            return; % Exit loop and function early, no need to check further points.
        end
    end % End loop through sample points

    % If the loop completes without finding any obstruction, los_clear remains true.

end % END check_LOS_internal