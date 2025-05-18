% =========================================================================
% KEYBOARD INPUT CONTROLLER (FINAL - Corrected, Re-Commented)
% Version: FINAL 1.2
%
% Description:
%   This function polls the keyboard associated with a specified MATLAB figure
%   window for user flight control inputs (pitch and roll adjustments). It
%   detects key presses for arrow keys or WASD keys. Includes a timeout
%   feature: if no valid control key is pressed within a configured duration
%   (`config.simulation.realtime_input_timeout_s`), it signals that the user
%   input is inactive (`isActive = false`), allowing the simulation to revert
%   to autopilot control. Designed to be robust against empty key buffers or
%   invalid/missing figure handles.
%
% Usage:
%   Called repeatedly within the main simulation loop (typically from
%   `aircraft_dynamics_model.m`).
%   Example Call:
%     `[p_adj, r_adj, active] = keyboard_input_controller(config, fig_handle_for_input);`
%
% Inputs:
%   config       - The main configuration structure (uses fields from
%                  `config.simulation` and `config.advanced`).
%   current_fig  - Handle to the MATLAB figure window that should capture
%                  keyboard input. This figure must have focus for keys to be
%                  registered by `get(..., 'CurrentCharacter')`. Can be empty
%                  or invalid, in which case input is gracefully disabled.
%
% Outputs:
%   pitch_adjust - Requested change in pitch based on key press.
%                  Value is +PITCH_STEP for Up, -PITCH_STEP for Down, 0 otherwise.
%                  Units depend on PITCH_STEP (typically interpreted as degrees).
%   roll_adjust  - Requested change in roll based on key press.
%                  Value is +ROLL_STEP for Right, -ROLL_STEP for Left, 0 otherwise.
%                  Units depend on ROLL_STEP (typically interpreted as degrees).
%   isActive     - Boolean flag:
%                  `true`: User pressed a control key this step OR within the timeout period.
%                          The simulation should likely use pitch/roll_adjust values.
%                  `false`: No control key pressed recently (timeout expired or never pressed).
%                           The simulation should likely use autopilot commands.
%
% Key Mapping:
%   - Up Arrow / 's': Pitch Up (+PITCH_STEP adjustment command)
%   - Down Arrow / 'w': Pitch Down (-PITCH_STEP adjustment command)
%   - Left Arrow / 'a': Roll Left (-ROLL_STEP adjustment command)
%   - Right Arrow / 'd': Roll Right (+ROLL_STEP adjustment command)
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-04] - Corrected toc error handling
% =========================================================================
function [pitch_adjust, roll_adjust, isActive] = keyboard_input_controller(config, current_fig)

    % --- Persistent Variable ---
    % Stores the time marker (uint64) from the last valid control key press,
    % obtained using `tic`. This variable retains its value across multiple
    % calls to this function within a single MATLAB session.
    persistent last_key_press_tic_val; % Renamed from last_key_press_time for clarity on type

    % Initialize only once when the function is first loaded or if 'clear functions' was called.
    % Use uint64(0) as an indicator that no valid key has been pressed yet.
    if isempty(last_key_press_tic_val)
        last_key_press_tic_val = uint64(0);
    end

    % --- Initialize Default Outputs for This Call ---
    pitch_adjust = 0;   % No pitch adjustment requested by default
    roll_adjust  = 0;   % No roll adjustment requested by default
    isActive     = false; % Assume user is inactive by default

    % --- Get Configuration Parameters ---
    % Timeout duration (seconds). If <= 0, timeout is effectively disabled (active only on key press).
    timeout_duration = config.simulation.realtime_input_timeout_s;
    % Step size for pitch/roll adjustments (degrees per detected key press event).
    PITCH_STEP = 1.0; % Degrees
    ROLL_STEP  = 1.0; % Degrees

    % --- Capture Keyboard Input ---
    key = ''; % Initialize captured key variable for this step

    % Check if a valid figure handle was provided for input capture.
    % `ishandle` checks if it's a valid graphics object handle.
    % `strcmp(get(...))` checks if it's specifically a figure.
    if ~isempty(current_fig) && ishandle(current_fig) && strcmp(get(current_fig,'Type'), 'figure')
        try
            % Retrieve the last character typed while the figure had focus.
            % Note: This only gets the *last* character. Rapid typing might miss keys.
            % For more robust input (handling multiple keys, key-up/down events),
            % a different approach using KeyPressFcn/KeyReleaseFcn callbacks might be needed,
            % but that adds complexity with state management.
            key = get(current_fig, 'CurrentCharacter');

            % IMPORTANT: Clear the figure's character buffer immediately after reading.
            % If not cleared, the same character might be processed again on the next call
            % if no other key is pressed in between.
            if ~isempty(key) % Only clear if a key was actually read
                 set(current_fig, 'CurrentCharacter', char(0)); % Use ASCII null or '' to clear
            end

        catch ME_getkey % Handle potential errors (e.g., figure closed, invalid handle)
            if config.advanced.VERBOSE_LEVEL >= 2 % Log error only in verbose mode
                 fprintf('Debug [Keyboard]: Error getting CurrentCharacter: %s\n', ME_getkey.message);
            end
            key = ''; % Ensure key is empty on error
        end
    else
        % Handle case where no valid figure handle is available
        if config.simulation.ENABLE_REALTIME_INPUT && config.advanced.VERBOSE_LEVEL >= 2
             % Print debug message if input expected but no figure available
             % fprintf('Debug [Keyboard]: No valid figure handle provided for keyboard input.\n');
        end
        key = ''; % Ensure key is empty
    end

    % --- Process Captured Key Press ---
    key_pressed_this_step = false; % Flag if a *valid control key* was processed this step

    % Check if a single, valid character was captured
    if ~isempty(key) && ischar(key) && length(key) == 1
        key_code = double(key); % Get ASCII value for reliable comparison (esp. for arrow keys)

        % Compare key code against defined control keys (Arrows and WASD)
        switch key_code
            % --- Pitch Control ---
            case {30, double('s')} % 30 = Up Arrow ASCII code; also allow 's'
                pitch_adjust = PITCH_STEP;      % Request Pitch Up command
                key_pressed_this_step = true;   % Mark that a control key was pressed
                % Debug message option
                % if config.advanced.VERBOSE_LEVEL >= 2, fprintf('Debug [Keyboard]: Key Press -> Pitch Up\n'); end

            case {31, double('w')} % 31 = Down Arrow ASCII code; also allow 'w'
                pitch_adjust = -PITCH_STEP;     % Request Pitch Down command
                key_pressed_this_step = true;
                % if config.advanced.VERBOSE_LEVEL >= 2, fprintf('Debug [Keyboard]: Key Press -> Pitch Down\n'); end

            % --- Roll Control ---
            case {28, double('a')} % 28 = Left Arrow ASCII code; also allow 'a'
                roll_adjust = -ROLL_STEP;       % Request Roll Left command
                key_pressed_this_step = true;
                % if config.advanced.VERBOSE_LEVEL >= 2, fprintf('Debug [Keyboard]: Key Press -> Roll Left\n'); end

            case {29, double('d')} % 29 = Right Arrow ASCII code; also allow 'd'
                roll_adjust = ROLL_STEP;        % Request Roll Right command
                key_pressed_this_step = true;
                % if config.advanced.VERBOSE_LEVEL >= 2, fprintf('Debug [Keyboard]: Key Press -> Roll Right\n'); end

            otherwise
                % Key pressed was not one of the designated control keys.
                key_pressed_this_step = false; % Ensure flag is false
                % Optional: Log ignored keys if debugging input issues
                % if config.advanced.VERBOSE_LEVEL >= 2
                %     fprintf('Debug [Keyboard]: Ignored Key Press: %s (Code: %d)\n', key, key_code);
                % end
        end % end switch
    end % end if ~isempty(key)

    % --- Update Activity Status based on Key Press and Timeout ---
    if key_pressed_this_step
        % If a valid control key was pressed *this step*:
        % 1. Record the current time marker using `tic`.
        last_key_press_tic_val = tic;
        % 2. Set the output flag to active.
        isActive = true;
        % Optional debug message:
        % if config.advanced.VERBOSE_LEVEL >= 2, fprintf('Debug [Keyboard]: Active (Control Key Pressed)\n'); end

    elseif timeout_duration > 0
        % If no control key was pressed *this step*, AND timeout is enabled:
        % Check if enough time has passed since the *last* recorded control key press.

        % ** CRITICAL FIX **: Only call `toc` if `last_key_press_tic_val` holds a valid timer value
        % (i.e., not the initial uint64(0)). Calling toc(0) or toc(-inf) causes an error.
        if last_key_press_tic_val ~= uint64(0)
            % A valid key was pressed previously, calculate time elapsed since then.
            time_since_last_press = toc(last_key_press_tic_val);

            % Compare elapsed time to the timeout duration.
            if time_since_last_press < timeout_duration
                % Still within the timeout window, user is considered active.
                isActive = true;
                % Optional debug message:
                % if config.advanced.VERBOSE_LEVEL >= 2, fprintf('Debug [Keyboard]: Active (Within Timeout: %.2f s / %.1f s)\n', time_since_last_press, timeout_duration); end
            else
                % Timeout has expired, user is now considered inactive.
                isActive = false; % Default value is already false, but set explicitly for clarity
                % Optional debug message:
                % if config.advanced.VERBOSE_LEVEL >= 2, fprintf('Debug [Keyboard]: Inactive (Timeout Expired: %.2f s >= %.1f s)\n', time_since_last_press, timeout_duration); end
            end
        else
            % No valid control key has ever been pressed in this session.
            % Therefore, the timeout condition is met by default.
            isActive = false; % Remain inactive
            % Optional debug message:
            % if config.advanced.VERBOSE_LEVEL >= 2, fprintf('Debug [Keyboard]: Inactive (No valid key press recorded yet)\n'); end
        end
    else
        % If timeout_duration is zero or negative, timeout is disabled.
        % isActive is only true if a key was pressed *this specific step*.
        % Since key_pressed_this_step is false here, isActive remains false.
         isActive = false;
    end

    % Note: The returned pitch/roll adjustments are simple step changes (+1, -1, 0 multiplied by STEP).
    % The `aircraft_dynamics_model` is responsible for interpreting these adjustments
    % (e.g., adding to current target, using as rate command) and applying aircraft
    % rate and angle limits.

end % END OF FUNCTION keyboard_input_controller