function set_figure_style_helper_local(fig_handle, theme, is_cyberpunk)
    % Applies background color based on theme.
    if is_cyberpunk
        set(fig_handle, 'Color', theme.bgColor);
    else
        set(fig_handle, 'Color', [0.94 0.94 0.94]); % MATLAB default grey
    end
end