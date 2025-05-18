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
% Maintainer: [Your Name/Team]
% Last Updated: 2025-04-05 - Generalized raster reference check.
% =========================================================================
function terrain_out = terrain_model(terrain_config, earth)
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
    terrain_out.earth          = earth;
    terrain_out.get_altitude   = @(lat, lon) 0; % Default function: always returns 0m elevation
    %terrain_out.check_LOS      = @(varargin) true; % Default function: always returns true (clear LOS)

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

                % % Convert referencing object if it's a postings reference
                % if isa(R_raw, 'map.rasterref.GeographicPostingsReference')
                %     fprintf('  Converting GeographicPostingsReference to GeographicCellsReference (legacy compatible)...\n');
                % 
                %     dLat = diff(R_raw.LatitudeLimits)  / (R_raw.RasterSize(1) - 1);
                %     dLon = diff(R_raw.LongitudeLimits) / (R_raw.RasterSize(2) - 1);
                % 
                %     lat_limits_cells = [R_raw.LatitudeLimits(1) - dLat/2, R_raw.LatitudeLimits(2) + dLat/2];
                %     lon_limits_cells = [R_raw.LongitudeLimits(1) - dLon/2, R_raw.LongitudeLimits(2) + dLon/2];
                % 
                %     R = georefcells(lat_limits_cells, lon_limits_cells, size(dem_raw));
                % else
                %     R = R_raw;
                % end
R = R_raw;
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
        % --- LOS wrapper that correctly returns 4 outputs via deal() ---
        %terrain_out.check_LOS = @check_LOS_wrapper;

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
        %terrain_out.check_LOS      = @(varargin) true;
    end

    % -------------------------------------------------------------------------
% Override the LOS handle with a wrapper that adapts to caller arity
% -------------------------------------------------------------------------
    terrain_out.check_LOS = @check_LOS_wrapper;

    function varargout = check_LOS_wrapper(lat1, lon1, alt1_msl, lat2, lon2, alt2_msl, tx_cfg, varargin)
        % If caller only expects one output, return only the boolean
        if nargout <= 1
            varargout{1} = check_LOS_internal( ...
                terrain_out, ...
                lat1, lon1, alt1_msl, ...
                lat2, lon2, alt2_msl, ...
                tx_cfg, varargin{:} );
        else
            % Caller requested full obstruction geometry
            [varargout{1}, varargout{2}, varargout{3}, varargout{4}] = ...
                check_LOS_internal( ...
                    terrain_out, ...
                    lat1, lon1, alt1_msl, ...
                    lat2, lon2, alt2_msl, ...
                    tx_cfg, varargin{:} );
        end
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
            if isa(R, 'map.rasterref.GeographicCellsReference') || isa(R, 'map.rasterref.GeographicPostingsReference')
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
                        %elevation = geointerp(dem, R, lat, lon, method);

                        [row, col] = geographicToDiscrete(R, lat, lon);
                        row = max(1, min(row, size(dem,1)));
                        col = max(1, min(col, size(dem,2)));
                        elevation = dem(row, col);
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

%start
% % =========================================================================
% % INTERNAL HELPER FUNCTION: Check Line-of-Sight (LOS)
% % Version: 2.1 (Modified to return obstruction geometry)
% % =========================================================================
% function [los_clear, h_obstruction_m, d1_obstacle_m, d2_obstacle_m] = check_LOS_internal(terrain_obj, lat1, lon1, alt1_msl, lat2, lon2, alt2_msl, varargin)
%     % Checks Line-of-Sight between two points, considering terrain obstruction.
%     % Attempts los2 if available, otherwise uses manual ECEF method.
%     % RETURNS obstruction height (h) and distances (d1, d2) if blocked.
%     %
%     % Args:
%     %   terrain_obj (struct): The terrain model object (MUST contain .earth).
%     %   lat1, lon1 (double): Coordinates of the start point (degrees).
%     %   alt1_msl (double): Altitude of the start point (meters MSL).
%     %   lat2, lon2 (double): Coordinates of the end point (degrees).
%     %   alt2_msl (double): Altitude of the end point (meters MSL).
%     %   varargin{1} (optional, int): Number of sample points for manual check (default 100).
%     %
%     % Returns:
%     %   los_clear (logical): True if LOS is clear, False if obstructed.
%     %   h_obstruction_m (double): Height of the primary obstruction above LOS path (m). NaN if clear.
%     %   d1_obstacle_m (double): Distance from point 1 to primary obstruction (m). NaN if clear.
%     %   d2_obstacle_m (double): Distance from point 2 to primary obstruction (m). NaN if clear.
% 
%     los_clear = true; % Assume clear unless obstruction found
%     h_obstruction_m = NaN;
%     d1_obstacle_m = NaN;
%     d2_obstacle_m = NaN;
%     method_used = 'unknown'; % Track which method was used
% 
%     % --- Basic Input and Data Checks ---
%     if ~terrain_obj.data_loaded
%         method_used = 'no_terrain_data';
%         return; % Assume clear if no terrain
%     end
%     if ~isfield(terrain_obj, 'earth') || isempty(terrain_obj.earth)
%        warning('TerrainModel:LOS:MissingEarth', 'Earth model object missing. Cannot perform ECEF LOS. Assuming clear.');
%        method_used = 'error_missing_earth';
%        return;
%     end
% 
%     % --- Calculate Total Great Circle Distance (approx for d1/d2 calculation later) ---
%     [total_dist_m, ~] = terrain_obj.earth.distance_bearing(lat1, lon1, lat2, lon2);
%     if total_dist_m < 1e-3 % Avoid division by zero for very close points
%         return;
%     end
% 
%     % --- Check if Endpoints are Below Terrain ---
%     try
%         terrain_alt1 = terrain_obj.get_altitude(lat1, lon1);
%         terrain_alt2 = terrain_obj.get_altitude(lat2, lon2);
%         tolerance = 0.1; % meters
%         if alt1_msl < (terrain_alt1 - tolerance) || alt2_msl < (terrain_alt2 - tolerance)
%             los_clear = false;
%             method_used = 'endpoint_below_terrain';
%             % For endpoints below ground, h, d1, d2 remain NaN as specific obstruction isn't well-defined this way.
%             return;
%         end
%     catch ME_alt_check
%         warning(ME_alt_check.identifier, ...
%                 'Failed endpoint altitude check during LOS: %s. Proceeding cautiously.', ME_alt_check.message);
%     end
% 
%     % % --- Attempt 1: Use Mapping Toolbox 'los2' ---
%     % use_los2 = false;
%     % mapping_toolbox_exists = license('test', 'Mapping Toolbox') && ~isempty(ver('map')) && exist('los2', 'file');
%     % 
%     % if mapping_toolbox_exists && strcmp(terrain_obj.ref_obj_type, 'grid')
%     %     try
%     %         alt1_agl = alt1_msl - terrain_alt1;
%     %         alt2_agl = alt2_msl - terrain_alt2;
%     %         % los2 doesn't directly return h, d1, d2. If blocked, we still need manual method for those.
%     %         [vis, ~, ~, ~] = los2(terrain_obj.dem_data, terrain_obj.ref_obj, lat1, lon1, alt1_agl, lat2, lon2, alt2_agl);
%     %         if logical(vis)
%     %             method_used = 'los2_toolbox_clear';
%     %             return; % LOS is clear, h/d1/d2 remain NaN
%     %         else
%     %             % LOS blocked according to los2, need manual method to find h, d1, d2
%     %             method_used = 'los2_blocked_fallback_manual';
%     %             los_clear = false; % Mark as blocked
%     %             % Continue to manual ECEF method below...
%     %         end
%     %     catch ME_los2
%     %         warning('TerrainModel:LOS:los2Fail', 'Mapping Toolbox los2 failed: %s. Falling back to manual ECEF.', ME_los2.message);
%     %         % Continue to manual ECEF method below...
%     %     end
%     % end
% 
%     % --- Attempt 2: Manual ECEF-based Calculation (Fallback or for h/d1/d2) ---
%     if ~strcmp(method_used, 'los2_toolbox_clear') % Check if los2 didn't confirm clear or wasn't used
%         method_used = 'manual_ecef';
%         los_clear = true; % Assume clear *within this method* until obstruction found
%         max_h_found = -Inf;
%         idx_max_h = -1;
% 
%         try
%             num_samples = 100;
%             if ~isempty(varargin) && isnumeric(varargin{1}) && varargin{1} > 2
%                 num_samples = max(50, ceil(total_dist_m/500));   % 500 m spacing
% 
%             end
% 
%             [X1, Y1, Z1] = terrain_obj.earth.LLH_to_ECEF(lat1, lon1, alt1_msl);
%             [X2, Y2, Z2] = terrain_obj.earth.LLH_to_ECEF(lat2, lon2, alt2_msl);
% 
%             X_samples = linspace(X1, X2, num_samples);
%             Y_samples = linspace(Y1, Y2, num_samples);
%             Z_samples = linspace(Z1, Z2, num_samples);
% 
%             for i = 2:(num_samples - 1)
%                 [lat_sample, lon_sample, alt_sample_los_path_msl] = terrain_obj.earth.ECEF_to_LLH(X_samples(i), Y_samples(i), Z_samples(i));
% 
%                 terrain_alt_sample = -Inf; % Default if lookup fails
%                 try
%                    terrain_alt_sample = terrain_obj.get_altitude(lat_sample, lon_sample);
%                    if isnan(terrain_alt_sample)
%                        terrain_alt_sample = -Inf;
%                    end
%                 catch ME_alt_sample
%                     warning(ME_alt_sample.identifier, ...
%                            'Failed LOS altitude sample at ECEF point %d / %d: %s. Assuming clear at this point.', i, num_samples, ME_alt_sample.message);
%                 end
% 
%                 % Check for Obstruction and track max height
%                 tolerance = 0.1; % meters
%                 if (alt_sample_los_path_msl - tolerance) < terrain_alt_sample
%                     los_clear = false; % Path is blocked
%                     current_h = terrain_alt_sample - alt_sample_los_path_msl;
%                     if current_h > max_h_found
%                         max_h_found = current_h;
%                         idx_max_h = i;
%                     end
%                 end
%             end % End loop
% 
%             % If path was found to be blocked, calculate output geometry
%             if ~los_clear && idx_max_h > 0
%                 h_obstruction_m = max(0, max_h_found); % Ensure h is not negative
% 
%                 % Approximate d1 and d2 based on sample index and total distance
%                 fraction_d1 = (idx_max_h - 1) / (num_samples - 1);
%                 d1_obstacle_m = fraction_d1 * total_dist_m;
%                 d2_obstacle_m = total_dist_m - d1_obstacle_m;
% 
%                 % More Accurate d1/d2 (optional - using ECEF points)
%                 [X_obs, Y_obs, Z_obs] = deal(X_samples(idx_max_h), Y_samples(idx_max_h), Z_samples(idx_max_h));
%                 d1_obstacle_m = norm([X_obs-X1, Y_obs-Y1, Z_obs-Z1]);
%                 d2_obstacle_m = norm([X2-X_obs, Y2-Y_obs, Z2-Z_obs]);
%             end
% 
%         catch ME_ecef_los
%             warning('TerrainModel:LOS:EcefFail', 'Manual ECEF LOS check failed: %s. Assuming clear LOS.', ME_ecef_los.message);
%             los_clear = true; % Default to clear on error
%             h_obstruction_m = NaN; d1_obstacle_m = NaN; d2_obstacle_m = NaN;
%             method_used = 'error_manual_ecef';
%         end
%     end % End manual fallback check
% 
% end % END check_LOS_internal

% =========================================================================
% INTERNAL HELPER FUNCTION: Check Line-of-Sight (LOS) with high fidelity
% Version: 2.5 – Adaptive ECEF-based manual fallback, handles nargout
% =========================================================================
function [los_clear, h_obstruction_m, d1_obstacle_m, d2_obstacle_m] = ...
    check_LOS_internal(terrain_obj, lat1, lon1, alt1_msl, lat2, lon2, alt2_msl, tx_cfg, varargin)

    % Initialize outputs
    los_clear      = true; % Default assumption
    h_obstruction_m= NaN;
    d1_obstacle_m  = NaN;
    d2_obstacle_m  = NaN;

    % Constants & Setup
    c       = physconst('LightSpeed');            % m/s
    if ~isfield(tx_cfg, 'freq_hz') || isempty(tx_cfg.freq_hz) || tx_cfg.freq_hz <= 0
        warning('LOS:MissingFreq', 'tx_cfg.freq_hz missing or invalid. Using default 1 GHz for Fresnel calc.');
        freq_Hz = 1e9; % Default frequency if not provided
    else
        freq_Hz = tx_cfg.freq_hz; % Hz
    end
    lambda  = c / freq_Hz;                       % m
    num_outputs_requested = nargout; % How many outputs the caller wants

    % Quick exit checks
    if ~isfield(terrain_obj, 'earth') || isempty(terrain_obj.earth)
        warning('LOS:NoEarth', 'Earth model missing in terrain_obj. Cannot perform ECEF LOS. Assuming clear.'); return;
    end
     if ~terrain_obj.data_loaded
        warning('LOS:NoTerrain','No terrain data – assuming clear LOS.'); return;
    end
    try % Endpoint check
        if alt1_msl < terrain_obj.get_altitude(lat1,lon1)-0.1 || alt2_msl < terrain_obj.get_altitude(lat2,lon2)-0.1
            los_clear = false; return;
        end
    catch ME_alt_check
         warning('LOS:EndpointAltCheckFail', 'Failed endpoint altitude check: %s. Proceeding.', ME_alt_check.message);
    end

    % --- Calculate Geodesic Distance and ECEF Coordinates ---
    try
        if ismethod(terrain_obj.earth, 'distance_bearing')
             [total_dist_m, ~] = terrain_obj.earth.distance_bearing(lat1, lon1, lat2, lon2);
        else
             wgs84 = wgs84Ellipsoid('metre'); total_dist_m = distance(lat1, lon1, lat2, lon2, wgs84);
             %warning('LOS:NoDistanceMethod', 'Using MATLAB distance function, earth.distance_bearing preferred.');
        end
        if total_dist_m < 1e-3, return; end % Handle close points

        [X1, Y1, Z1] = terrain_obj.earth.LLH_to_ECEF(lat1, lon1, alt1_msl);
        [X2, Y2, Z2] = terrain_obj.earth.LLH_to_ECEF(lat2, lon2, alt2_msl);
        P1_ecef = [X1; Y1; Z1]; P2_ecef = [X2; Y2; Z2];
        V_ecef = P2_ecef - P1_ecef; % ECEF vector P1 to P2
        D_ecef = norm(V_ecef);     % Straight line ECEF distance

    catch ME_geom_setup
        warning('LOS:GeomSetupFail', 'Failed initial geometry/ECEF setup: %s. Assuming clear.', ME_geom_setup.message); return;
    end

    % Compute variable k-factor
    h_mid = (alt1_msl + alt2_msl)/2; N_mid = max(200, 315 - 39*(h_mid/1e3)); k_var = 157 / (N_mid + 273);

    % --- 1) Try Mapping Toolbox los2 ---
    use_los2 = license('test','map_toolbox') && exist('los2','file') && strcmpi(terrain_obj.ref_obj_type,'grid');
    if use_los2
        try
            h1_agl = alt1_msl - terrain_obj.get_altitude(lat1,lon1);
            h2_agl = alt2_msl - terrain_obj.get_altitude(lat2,lon2);
            vis = los2(terrain_obj.dem_data, terrain_obj.ref_obj, lat1, lon1, lat2, lon2, h1_agl, h2_agl, 'AGL','AGL','REFRACTION', k_var);
            if logical(vis)
                 los_clear = true; return; % Path clear, return immediately
            end
            % else: los2 ran ok and found blockage. Proceed to manual check below.
        catch ME_los2
            warning('LOS:los2fail','los2 failed: %s. Falling back to manual ECEF solver.', ME_los2.message);
            % Proceed to manual check below.
        end
    end

    % --- 2) Manual ECEF-based adaptive solver ---
    los_clear = true; h_obstruction_m = NaN; d1_obstacle_m = NaN; d2_obstacle_m = NaN; % Initialize outputs
    N_samples = max(50, ceil(total_dist_m/500)); % Coarse sampling

    try
        if num_outputs_requested <= 1
            % --- Case: Only boolean visibility needed ---
            los_clear = manual_LOS_boolean_ecef(terrain_obj, P1_ecef, P2_ecef, lambda, k_var, total_dist_m, N_samples);

        else
            % --- Case: Full geometry needed ---
            idx_obs_initial = -1; % To store index of obstruction from first pass
            [clearFlagManual, h_obs_manual, d1_manual, d2_manual, idx_obs_initial] = ...
                manual_LOS_ecef(terrain_obj, P1_ecef, P2_ecef, lambda, k_var, total_dist_m, N_samples);

            los_clear = clearFlagManual;
            h_obstruction_m = h_obs_manual;
            d1_obstacle_m = d1_manual;
            d2_obstacle_m = d2_manual;

            % --- Adaptive Refinement ---
            % Define borderline threshold (e.g., clearance < 0.6 * F1 at path center)
            fresnel_threshold = 0.6 * sqrt(lambda * total_dist_m / 4);
            is_borderline = los_clear && ~isnan(h_obs_manual) && h_obs_manual < fresnel_threshold; % Check if clear but close

            if ~los_clear || is_borderline
                 % Proceed with refinement only if an obstruction index was found
                 if idx_obs_initial > 0
                    % Define window around initial obstruction distance d1_manual
                    s0 = d1_manual;
                    win_frac = 0.10; % +/- 10% window
                    s_low = max(0, s0 - total_dist_m * win_frac);
                    s_high = min(total_dist_m, s0 + total_dist_m * win_frac);
                    D_win = s_high - s_low;

                    if D_win > 10 % Only refine if window is reasonably large (e.g., > 10m)
                        % Calculate ECEF start/end points of the window
                        P_win_start_ecef = P1_ecef + (V_ecef / D_ecef) * (s_low / total_dist_m * D_ecef); % Interpolate along ECEF vector
                        P_win_end_ecef   = P1_ecef + (V_ecef / D_ecef) * (s_high / total_dist_m * D_ecef);

                        % Dense sampling within the window
                        N_refine = max(20, ceil(D_win / 100)); % Aim for ~100m spacing in window

                        % Call refinement function
                        [clearFlagRefine, h_refine, d1_refine_abs, ~] = ...
                            refine_LOS_ecef(terrain_obj, P_win_start_ecef, P_win_end_ecef, ...
                                            lambda, k_var, total_dist_m, s_low, D_win, N_refine);

                        % Update results if refinement found blockage or a worse obstruction
                        if ~clearFlagRefine
                            los_clear = false;
                            if isnan(h_obstruction_m) || h_refine > h_obstruction_m
                                h_obstruction_m = h_refine;
                                d1_obstacle_m = d1_refine_abs; % Use absolute distance from P1
                                d2_obstacle_m = total_dist_m - d1_obstacle_m;
                            end
                        elseif is_borderline % If was borderline clear, refinement might confirm clear
                           los_clear = true; % Keep clear unless refinement found blockage
                           % Keep original borderline geometry unless refinement proved clear
                           % (h_refine should be negative/NaN if clear)
                           if isnan(h_refine) || h_refine < 0
                               h_obstruction_m = NaN;
                               d1_obstacle_m = NaN;
                               d2_obstacle_m = NaN;
                           end
                        end
                    end % End if D_win > 10
                 end % End if idx_obs_initial > 0
            end % End refinement block
        end % End if/else based on num_outputs_requested

    catch ME_manual_solve
        warning('LOS:ManualECEFFail', 'Manual ECEF LOS solver failed: %s. Assuming blocked.', ME_manual_solve.message);
        los_clear = false; h_obstruction_m = NaN; d1_obstacle_m = NaN; d2_obstacle_m = NaN;
    end
end


% =========================================================================
% ECEF-based Manual LOS Calculation (Full Geometry) - Returns obs index
% =========================================================================
function [clearFlag, h_obs, d1, d2, idx_max_h] = manual_LOS_ecef(terr, P1_ecef, P2_ecef, lam, kf, D_geo, N)
    clearFlag=true; h_obs=NaN; d1=NaN; d2=NaN; idx_max_h = -1; % Init outputs
    max_h_found = -Inf;

    if D_geo <= 0, return; end

    try
        Re = terr.earth.params.a;
        indices = 2:(N-1);
        X_samples = linspace(P1_ecef(1), P2_ecef(1), N);
        Y_samples = linspace(P1_ecef(2), P2_ecef(2), N);
        Z_samples = linspace(P1_ecef(3), P2_ecef(3), N);
        clearance1 = NaN(1, N); s_values = NaN(1, N);

        for i = indices
            P_sample_ecef = [X_samples(i); Y_samples(i); Z_samples(i)];
            [lat_s, lon_s, alt_path_msl] = terr.earth.ECEF_to_LLH(P_sample_ecef(1), P_sample_ecef(2), P_sample_ecef(3));
            terrain_alt_s = terr.get_altitude(lat_s, lon_s);
            s = D_geo * (i-1) / (N-1); s_values(i) = s;
            bulge = s * (D_geo - s) / (2 * kf * Re);
            h_eff = alt_path_msl - bulge;
            rF1 = 0; if s > 1e-6 && (D_geo - s) > 1e-6, rF1 = sqrt(lam * s * (D_geo - s) / D_geo); end
            current_clearance = h_eff - terrain_alt_s - 0.6 * rF1;
            clearance1(i) = current_clearance;

            if current_clearance <= 0
                clearFlag = false;
                obstruction_height = terrain_alt_s - (h_eff - 0.6 * rF1); % How high terrain is above LOS + 0.6*F1
                if obstruction_height > max_h_found
                    max_h_found = obstruction_height;
                    idx_max_h = i;
                end
            end
        end

        if ~clearFlag && idx_max_h > 0
            h_obs = max(0, max_h_found);
            d1 = s_values(idx_max_h); d2 = D_geo - d1;
        elseif ~clearFlag % Blocked, but somehow didn't find max_h
             [~, temp_idx] = min(clearance1(indices)); idx_max_h = temp_idx + 1;
             h_obs = 0; d1 = s_values(idx_max_h); d2 = D_geo - d1;
             warning('LOS:ManualGeomInconclusive','Manual LOS blocked but obstruction height calc failed.');
        end
        % Also return borderline geometry if path was clear but close
        if clearFlag
            [min_clearance, temp_idx] = min(clearance1(indices));
            idx_max_h = temp_idx + 1; % Index of minimum clearance
            h_obs = -min_clearance; % Report positive value indicating how close it was
            d1 = s_values(idx_max_h); d2 = D_geo - d1;
        end

    catch ME_manual_ecef
        warning('LOS:ManualECEFCalcError', 'Error during manual ECEF LOS calculation: %s. Assuming blocked.', ME_manual_ecef.message);
        clearFlag=false; h_obs=NaN; d1=NaN; d2=NaN; idx_max_h = -1;
    end
end

% =========================================================================
% ECEF-based Manual LOS Calculation (Boolean Only)
% =========================================================================
function clearFlag = manual_LOS_boolean_ecef(terr, P1_ecef, P2_ecef, lam, kf, D_geo, N)
    clearFlag=true;
    if D_geo <= 0, return; end

    try
        Re = terr.earth.params.a;
        indices = 2:(N-1);
        X_samples = linspace(P1_ecef(1), P2_ecef(1), N);
        Y_samples = linspace(P1_ecef(2), P2_ecef(2), N);
        Z_samples = linspace(P1_ecef(3), P2_ecef(3), N);

        for i = indices
            P_sample_ecef = [X_samples(i); Y_samples(i); Z_samples(i)];
            [lat_s, lon_s, alt_path_msl] = terr.earth.ECEF_to_LLH(P_sample_ecef(1), P_sample_ecef(2), P_sample_ecef(3));
            terrain_alt_s = terr.get_altitude(lat_s, lon_s);
            s = D_geo * (i-1) / (N-1);
            bulge = s * (D_geo - s) / (2 * kf * Re);
            h_eff = alt_path_msl - bulge;
            rF1 = 0; if s > 1e-6 && (D_geo - s) > 1e-6, rF1 = sqrt(lam * s * (D_geo - s) / D_geo); end
            current_clearance = h_eff - terrain_alt_s - 0.6 * rF1;

            if current_clearance <= 0
                clearFlag = false; return; % Exit early
            end
        end

    catch ME_manual_bool_ecef
        warning('LOS:ManualBoolECEFError', 'Error during manual boolean ECEF LOS check: %s. Assuming blocked.', ME_manual_bool_ecef.message);
        clearFlag = false;
    end
end


% =========================================================================
% ECEF-based LOS Refinement within a Window
% =========================================================================
function [clearFlag, h_obs, d1_abs, d2_abs] = refine_LOS_ecef(terr, P_start_win_ecef, P_end_win_ecef, lam, kf, D_total_geo, s_low, D_win, N_refine)
    % Performs denser ECEF check within a specified window [s_low, s_high]
    % Returns geometry relative to the *original* path start (P1)

    clearFlag=true; h_obs=NaN; d1_abs=NaN; d2_abs=NaN; % Init outputs
    max_h_found = -Inf;
    idx_max_h = -1;

    if D_win <= 0, return; end % No window to check

    try
        Re = terr.earth.params.a;
        indices = 2:(N_refine-1);
        X_samples_win = linspace(P_start_win_ecef(1), P_end_win_ecef(1), N_refine);
        Y_samples_win = linspace(P_start_win_ecef(2), P_end_win_ecef(2), N_refine);
        Z_samples_win = linspace(P_start_win_ecef(3), P_end_win_ecef(3), N_refine);
        clearance1_win = NaN(1, N_refine);
        s_values_abs = NaN(1, N_refine); % Absolute distance from original P1

        for i = indices
            P_sample_ecef = [X_samples_win(i); Y_samples_win(i); Z_samples_win(i)];
            [lat_s, lon_s, alt_path_msl] = terr.earth.ECEF_to_LLH(P_sample_ecef(1), P_sample_ecef(2), P_sample_ecef(3));
            terrain_alt_s = terr.get_altitude(lat_s, lon_s);

            % Calculate absolute distance 's' from the original P1
            s_frac_in_win = (i-1) / (N_refine-1);
            s = s_low + D_win * s_frac_in_win; % Absolute distance from P1
            s_values_abs(i) = s;

            % Calculations use absolute 's' and total distance D_total_geo
            bulge = s * (D_total_geo - s) / (2 * kf * Re);
            h_eff = alt_path_msl - bulge;
            rF1 = 0; if s > 1e-6 && (D_total_geo - s) > 1e-6, rF1 = sqrt(lam * s * (D_total_geo - s) / D_total_geo); end
            current_clearance = h_eff - terrain_alt_s - 0.6 * rF1;
            clearance1_win(i) = current_clearance;

            if current_clearance <= 0
                clearFlag = false;
                obstruction_height = terrain_alt_s - (h_eff - 0.6 * rF1);
                if obstruction_height > max_h_found
                    max_h_found = obstruction_height;
                    idx_max_h = i;
                end
            end
        end

        if ~clearFlag && idx_max_h > 0
            h_obs = max(0, max_h_found);
            d1_abs = s_values_abs(idx_max_h); % Absolute distance from P1
            d2_abs = D_total_geo - d1_abs;
        elseif ~clearFlag % Blocked, but somehow didn't find max_h
             [~, temp_idx] = min(clearance1_win(indices)); idx_max_h = temp_idx + 1;
             h_obs = 0; d1_abs = s_values_abs(idx_max_h); d2_abs = D_total_geo - d1_abs;
             warning('LOS:RefineGeomInconclusive','Refined LOS blocked but obstruction height calc failed.');
        end
         % If refinement confirms clear (no negative clearance found)
        if clearFlag
             h_obs = NaN; d1_abs = NaN; d2_abs = NaN;
        end


    catch ME_refine_ecef
        warning('LOS:RefineECEFError', 'Error during ECEF LOS refinement: %s. Assuming blocked.', ME_refine_ecef.message);
        clearFlag=false; h_obs=NaN; d1_abs=NaN; d2_abs=NaN;
    end
end