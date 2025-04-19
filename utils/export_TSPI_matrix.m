% =========================================================================
% EXPORT TSPI MATRIX (FINAL - Re-Commented)
% Version: FINAL 1.0
%
% Description:
%   Saves the Time-Space-Position-Information (TSPI) matrix, which includes
%   aircraft position, attitude, speed, time, target waypoint, and potentially
%   Look/Depression angles, to a specified file format (CSV or MAT).
%
% Dynamic Header Handling:
%   - Uses the `tspi_headers` cell array (provided as input, typically from
%     `simulation_metadata`) to dynamically write the CSV header row.
%   - Assumes the columns in the `tspi_data` matrix correspond exactly
%     (in order and number) to the headers provided in `tspi_headers`.
%   - Dynamically creates the `fprintf` format specifier for writing CSV data
%     based on inferred data types from header names (e.g., '%.6f' for lat/lon).
%
% Usage:
%   Called from `main_simulation.m` after the simulation is complete.
%   `export_TSPI_matrix(tspi_data, tspi_headers, output_folder, output_format);`
%
% Inputs:
%   tspi_data       - NxM matrix containing the TSPI data log.
%   tspi_headers    - 1xM cell array of strings containing the header/name
%                     for each corresponding column in tspi_data.
%   output_folder   - String: Path to the main output directory for the run.
%   output_format   - String: Desired output format ('csv' or 'mat').
%   varargin        - Optional arguments (e.g., varargin{1}=true for partial save).
%
% Outputs:
%   Writes a 'tspi_data.csv' or 'tspi_data.mat' file to the output folder.
%   (Or 'tspi_data_PARTIAL.csv' if partial save indicated).
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-04]
% =========================================================================
function export_TSPI_matrix(tspi_data, tspi_headers, output_folder, output_format, varargin)

    % --- Input Validation ---
    if isempty(tspi_data)
        fprintf('[EXPORT] TSPI data is empty. Skipping export.\n');
        return;
    end
    if nargin < 4 || isempty(output_format)
        output_format = 'csv'; % Default to CSV format
        fprintf('[EXPORT] Output format not specified, defaulting to CSV.\n');
    end
     if nargin < 3 || isempty(output_folder) || ~ischar(output_folder)
         error('Output folder path (string) is required for TSPI export.');
     end
     if nargin < 2 || isempty(tspi_headers) || ~iscellstr(tspi_headers) % Use iscellstr for robustness
         error('TSPI headers (cell array of strings) are required for export.');
     end
     if size(tspi_data, 2) ~= length(tspi_headers)
          error('Number of columns in tspi_data (%d) does not match the number of provided headers (%d). Cannot export.', size(tspi_data, 2), length(tspi_headers));
     end

    % --- Prepare Output File Path ---
    file_basename = 'tspi_data';
    % Check for optional 'is_partial_save' flag
    if ~isempty(varargin) && islogical(varargin{1}) && varargin{1}
        file_basename = 'tspi_data_PARTIAL';
        fprintf('[EXPORT] Saving partial TSPI data.\n');
    end
    output_filepath_base = fullfile(output_folder, file_basename);

    % Ensure output folder exists (should be created by main_simulation, but check again)
    if ~isfolder(output_folder)
        try mkdir(output_folder);
        catch ME_dir
             error('Failed to create output folder for TSPI export: %s', ME_dir.message);
        end
    end

    % --- Export based on Format ---
    output_format = lower(output_format);

    if strcmp(output_format, 'csv')
        % --- Export to CSV ---
        csv_filepath = [output_filepath_base, '.csv'];
        fid = -1; % Initialize file identifier
        try
            % Open file for writing
            fid = fopen(csv_filepath, 'w');
            if fid == -1
                error('Could not open file for writing: %s', csv_filepath);
            end

            % Write dynamic header row (join headers with commas)
            fprintf(fid, '%s\n', strjoin(tspi_headers, ','));

            % Write data rows using a dynamically generated format string
            format_string = create_dynamic_format_spec_helper(tspi_headers); % Use helper
            % fprintf requires data to be column-major, so transpose tspi_data
            fprintf(fid, format_string, tspi_data');

            % Close the file
            fclose(fid);
            fprintf('[EXPORT] TSPI data exported to CSV: %s\n', csv_filepath);

        catch ME_csv % Catch errors during file writing
            if fid ~= -1, try fclose(fid); catch; end; end % Ensure file is closed on error
            error('Failed to export TSPI data to CSV "%s": %s', csv_filepath, ME_csv.message);
        end

    elseif strcmp(output_format, 'mat')
        % --- Export to MAT ---
        mat_filepath = [output_filepath_base, '.mat'];
        try
            % Save data and headers within a structure for better organization in the MAT file
            tspi_export_data = struct();
            tspi_export_data.data        = tspi_data;    % The N x M data matrix
            tspi_export_data.headers     = {tspi_headers}; % Store headers as cell array within struct
            tspi_export_data.description = 'Time-Space-Position Information from RF Link Simulation.';
            % Use -v7.3 flag for compatibility with potentially large data arrays
            save(mat_filepath, 'tspi_export_data', '-v7.3');
            fprintf('[EXPORT] TSPI data exported to MAT: %s\n', mat_filepath);
        catch ME_mat
             error('Failed to export TSPI data to MAT "%s": %s', mat_filepath, ME_mat.message);
        end
    else
        % Handle unsupported format
        warning('[EXPORT] Unsupported TSPI export format specified: "%s". Please use "csv" or "mat".', output_format);
    end

end % END OF FUNCTION export_TSPI_matrix


% =========================================================================
% INTERNAL HELPER: Create Dynamic fprintf Format Specifier for CSV
% =========================================================================
function format_string = create_dynamic_format_spec_helper(headers)
    % Creates an fprintf format string (e.g., '%.6f,%.6f,%.1f,...') based
    % on trying to guess appropriate formats from column header names.
    num_cols = length(headers);
    format_spec = cell(1, num_cols); % Initialize cell array for format specifiers

    for i = 1:num_cols
        hdr = lower(headers{i}); % Use lowercase header for comparisons

        % --- Guess Format Based on Header Content ---
        if contains(hdr, {'lat', 'lon'})        % Latitude/Longitude
            format_spec{i} = '%.7f'; % Higher precision often needed
        elseif contains(hdr, {'_m_msl','alt'})  % Altitude in meters
             format_spec{i} = '%.2f';
        elseif contains(hdr, {'_mps','speed'})  % Speeds in m/s
             format_spec{i} = '%.3f';
        elseif contains(hdr, {'_deg','head','pitch','roll','look','dep'}) % Angles in degrees
             format_spec{i} = '%.4f';
        elseif contains(hdr, 'time')            % Time in seconds
             format_spec{i} = '%.4f';
        elseif contains(hdr, {'index', 'flag', 'wp', 'id', 'num'}) % Integers or IDs
             format_spec{i} = '%d';
        else                                    % Default format for unknown columns
            format_spec{i} = '%.6g'; % General format, good precision
        end
    end

    % Join individual format specifiers with commas and add newline
    format_string = [strjoin(format_spec, ','), '\n'];
end