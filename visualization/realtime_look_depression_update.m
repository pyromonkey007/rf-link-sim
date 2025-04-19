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
