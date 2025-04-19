function set_axes_style_helper_local(ax_handle, theme, is_cyberpunk)
    % Applies colors, fonts, grid settings to axes based on theme.
    set(ax_handle, 'FontSize', theme.fontSize, 'FontName', theme.fontName);
    if is_cyberpunk
        set(ax_handle, 'Color',     theme.axesColor, ...         % Axes background
                       'XColor',    theme.axesTickColor, ...     % X-axis line, ticks, label color
                       'YColor',    theme.axesTickColor, ...     % Y-axis line, ticks, label color
                       'ZColor',    theme.axesTickColor, ...     % Z-axis line, ticks, label color (for 3D)
                       'GridColor', theme.gridColor, ...        % Major grid line color
                       'GridAlpha', 0.3);                     % Grid line transparency
                       % Optional: Minor grid styling
                       % 'MinorGridColor', theme.gridColor*0.8, 'MinorGridAlpha', 0.2);
    else % Standard MATLAB-like style
        set(ax_handle, 'Color',     'w', ...                    % White background
                       'XColor',    'k', 'YColor', 'k', 'ZColor', 'k', ... % Black axes lines/ticks/labels
                       'GridColor', [0.15 0.15 0.15], ...    % Dark grey grid
                       'GridAlpha', 0.15);
                       % 'MinorGridColor', [0.1 0.1 0.1], 'MinorGridAlpha', 0.1);
    end
end