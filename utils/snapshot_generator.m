% =========================================================================
% SNAPSHOT GENERATOR (FINAL - Re-Commented)
% Version: FINAL 1.0
%
% Description:
%   Captures the current visual state of a specified MATLAB figure window
%   and saves it as an image file (e.g., PNG, JPG) or a MATLAB FIG file
%   to the simulation output directory. Checks configuration flag before saving.
%
% Usage:
%   Typically called from visualization functions (`visualization_model.m`,
%   `tactical_replay_mode.m`, `ray_trace_visualizer.m`) after a plot is generated
%   or updated. Can also be called periodically from the main loop if needed.
%   `snapshot_generator(fig_handle, snapshot_name, config, sequence_num, file_format);`
%
% Inputs:
%   fig_handle     - Handle to the figure window to capture. Must be a valid figure handle.
%   snapshot_name  - String: Base name for the output file (e.g., 'trajectory_3d').
%                    A sequence number or timestamp will likely be appended.
%   config         - The main configuration structure (used to get output folder path
%                    and the ENABLE_SNAPSHOT_EXPORT flag).
%   sequence_num   - (Optional) Numeric or String identifier appended to the filename
%                    for uniqueness (e.g., frame number, time string). If empty,
%                    a timestamp is added.
%   file_format    - (Optional) String specifying the desired image format
%                    (e.g., 'png', 'jpg', 'tif', 'bmp', or 'fig' for MATLAB figure file).
%                    Defaults to 'png'.
%
% Patch 7 Integration:
%   - Checks `config.output.ENABLE_SNAPSHOT_EXPORT` flag. If false, the
%     function returns immediately without saving anything.
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-04]
% =========================================================================
function snapshot_generator(fig_handle, snapshot_name, config, sequence_num, file_format)

    % --- Input Validation and Defaults ---
    if nargin < 3
        error('Snapshot generator requires at least figure handle, base name, and config structure.');
    end
    % Check if fig_handle is a valid graphics handle of type 'figure'
    if ~ishandle(fig_handle) || ~strcmp(get(fig_handle, 'Type'), 'figure')
        warning('Invalid or deleted figure handle passed to snapshot_generator. Cannot save snapshot for "%s".', snapshot_name);
        return; % Exit if handle is invalid
    end
    if nargin < 4
        sequence_num = []; % Default: No sequence number provided
    end
    if nargin < 5 || isempty(file_format)
        file_format = 'png'; % Default to PNG format
    end
    file_format = lower(file_format); % Ensure lowercase format extension

    % --- Check Export Toggle (Patch 7) ---
    % Read the flag from the configuration structure.
    if ~isfield(config, 'output') || ~isfield(config.output, 'ENABLE_SNAPSHOT_EXPORT') || ~config.output.ENABLE_SNAPSHOT_EXPORT
        % Optional debug message if needed:
        % fprintf('Debug: Snapshot export disabled. Skipping save for "%s".\n', snapshot_name);
        return; % Exit the function immediately if snapshot export is disabled
    end

    % --- Determine Output Path ---
    output_folder = config.output.FOLDER; % Main output folder for the run
    figure_folder = fullfile(output_folder, 'figures'); % Specific subfolder for figures

    % Create the 'figures' subfolder if it doesn't already exist.
    if ~isfolder(figure_folder)
        try
            mkdir(figure_folder);
        catch ME_mkdir
            warning('Could not create figures subfolder: %s. Saving snapshot in main output folder: %s', figure_folder, ME_mkdir.message);
            figure_folder = output_folder; % Fallback to main output folder on error
        end
    end

    % --- Construct Unique Filename ---
    % Append sequence number or timestamp to the base snapshot name.
    if isempty(sequence_num)
        % If no sequence number, use a high-resolution timestamp for uniqueness
        filename_base = sprintf('%s_%s', snapshot_name, datestr(now, 'HHMMSSFFF'));
    elseif isnumeric(sequence_num)
        % If numeric, format with leading zeros (e.g., snapshot_0001)
        filename_base = sprintf('%s_%04d', snapshot_name, round(sequence_num)); % Ensure integer
    else
        % If string, use it directly (ensure it creates valid filenames)
        filename_base = sprintf('%s_%s', snapshot_name, char(sequence_num));
    end

    % Combine folder, base name, and file format extension.
    output_filename = fullfile(figure_folder, [filename_base, '.', file_format]);

    % --- Save the Figure ---
    try
        fprintf('  Saving snapshot: %s ...\n', output_filename);

        % Ensure the figure graphics are fully rendered before saving, especially
        % important if called immediately after plotting commands.
        drawnow;

        % Use different saving methods based on desired format.
        if strcmpi(file_format, 'fig')
            % Save as a MATLAB figure file (.fig), allowing later reopening and editing.
            savefig(fig_handle, output_filename);
        else
            % Save as a standard image format (PNG, JPG, etc.) using `print`.
            % `-d` flag specifies the device (format).
            % Optional: Add resolution argument like '-r300' for 300 DPI.
            print(fig_handle, output_filename, ['-d' file_format]); % Use default screen resolution
        end
        % Optional success message:
        % fprintf('  Snapshot saved successfully.\n');

    catch ME_save % Catch errors during the save operation
        warning('Failed to save snapshot "%s": %s', output_filename, ME_save.message);
        % Display more detailed error information if needed:
        % disp(ME_save.getReport);
    end

end % END OF FUNCTION snapshot_generator