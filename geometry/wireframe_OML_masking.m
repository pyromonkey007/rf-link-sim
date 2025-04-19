% =========================================================================
% WIREFRAME OML MASKING (FINAL - Re-Commented)
% Version: FINAL 1.0
%
% Description:
%   Implements an approximate Line-of-Sight (LOS) blocking check using a
%   2D wireframe silhouette (Outer Mold Line - OML) of the aircraft.
%   It projects the relevant 3D points onto the aircraft's body XY plane
%   and checks if the projected LOS line segment intersects the wireframe polygon.
%   This is faster but less accurate than 3D STL masking.
%
% Critical Input Requirement: Coordinate Frame Consistency!
%   - Assumes `tx_offset_body`, `rx_pos_local`, and `wireframe_points`
%     are all defined in the **aircraft body frame (X-fwd, Y-right, Z-down)**
%     relative to the aircraft Center of Gravity (CG).
%   - The calling function (`rf_propagation_model.m`) must provide correctly
%     transformed inputs.
%
% Method:
%   1. Takes Tx offset and Rx position (already in local body frame).
%   2. Takes 2D wireframe points (X,Y in body frame).
%   3. Projects the 3D Tx offset and Rx local position onto the 2D body XY plane.
%   4. Checks if the 2D line segment connecting projected Tx and Rx intersects
%      any edge of the 2D wireframe polygon.
%   5. Additionally checks if one point is inside the polygon and the other outside
%      (to catch cases where the segment doesn't cross an edge but still implies masking).
%
% Inputs:
%   tx_offset_body   - Transmitter position offset [X, Y, Z] relative to aircraft
%                      CG in the body frame (meters).
%   rx_pos_local     - Receiver position [X, Y, Z] transformed into the
%                      aircraft's local body frame (meters).
%   wireframe_points - Nx2 matrix defining the vertices [X, Y] of the 2D wireframe
%                      polygon in the aircraft body XY plane (meters). Must form a closed loop.
%   config_masking   - config.masking structure (passed for potential future use, unused currently).
%
% Output:
%   is_masked        - Boolean: True if the projected LOS intersects the wireframe, False otherwise.
%
% Dependencies:
%   - Requires MATLAB's `inpolygon` function for inside/outside check.
%   - Uses internal `line_segment_intersection_2d` helper function.
%
% Limitations:
%   - 2D approximation; ignores Z dimension relative to the aircraft body.
%   - Accuracy highly dependent on how well the 2D wireframe represents the actual 3D shape
%     for the specific Tx/Rx geometry. Best for side-view dominant masking.
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-04]
% =========================================================================
function is_masked = wireframe_OML_masking(tx_offset_body, rx_pos_local, wireframe_points, config_masking)
    % Inputs should be validated by the calling function (rf_propagation_model)

    is_masked = false; % Default: Assume not masked

    % --- Input Data Validation ---
    if isempty(wireframe_points) || size(wireframe_points, 2) ~= 2 || size(wireframe_points, 1) < 3
        warning('[WIRE MASK] Invalid wireframe_points data (must be Nx2 matrix, N>=3). Assuming no masking.');
        return;
    end
    if ~isnumeric(tx_offset_body) || ~isvector(tx_offset_body) || numel(tx_offset_body)~=3 || ...
       ~isnumeric(rx_pos_local) || ~isvector(rx_pos_local) || numel(rx_pos_local)~=3
        warning('[WIRE MASK] Invalid local Tx/Rx position inputs (must be 3-element numeric). Assuming no masking.');
        return;
    end

    % --- Project 3D Local Positions onto 2D Body XY Plane ---
    % Tx position projected is simply its body X and Y coordinates.
    tx_proj_2d = tx_offset_body(1:2); % [X_tx, Y_tx]

    % Rx position projected is its body X and Y coordinates.
    rx_proj_2d = rx_pos_local(1:2);   % [X_rx, Y_rx]

    % --- Check for Intersection between Projected LOS Segment and Wireframe Polygon ---
    % The projected LOS segment goes from tx_proj_2d to rx_proj_2d.
    line_start = tx_proj_2d(:)'; % Ensure row vector [1x2]
    line_end   = rx_proj_2d(:)'; % Ensure row vector [1x2]

    % Iterate through each edge of the wireframe polygon.
    num_wf_points = size(wireframe_points, 1);
    for i = 1:num_wf_points
        % Get the vertices of the current polygon edge.
        p1 = wireframe_points(i, :);                           % Current vertex [X, Y]
        p2 = wireframe_points(mod(i, num_wf_points) + 1, :); % Next vertex (wraps around using mod)

        % Check if the projected LOS segment intersects this polygon edge.
        if line_segment_intersection_2d(line_start, line_end, p1, p2) % Use internal helper
            is_masked = true; % Intersection detected
            % Optional debug message:
            % fprintf('Debug [WIRE MASK]: Intersection found with wireframe edge %d.\n', i);
            return; % Exit the function early, LOS is blocked
        end
    end % End loop through polygon edges

    % --- Additional Check: One Point Inside, One Outside ---
    % The segment intersection check might miss cases if the start/end points
    % are perfectly aligned with polygon vertices or have floating point issues.
    % This check covers cases where the LOS must logically cross the boundary
    % because one point is inside the wireframe and the other is outside.
    try
        tx_inside = inpolygon(tx_proj_2d(1), tx_proj_2d(2), wireframe_points(:,1), wireframe_points(:,2));
        rx_inside = inpolygon(rx_proj_2d(1), rx_proj_2d(2), wireframe_points(:,1), wireframe_points(:,2));

        % If one is inside and the other is outside, it implies masking.
        if tx_inside ~= rx_inside
            is_masked = true;
            % Optional debug message:
            % fprintf('Debug [WIRE MASK]: Masked because one point inside polygon, other outside (TxIn=%d, RxIn=%d).\n', tx_inside, rx_inside);
        end
        % Note: If both points are inside (tx_inside && rx_inside), is_masked remains false (correct behavior).
    catch ME_inpoly
        warning('[WIRE MASK] Error during inpolygon check: %s. Masking state may be inaccurate.', ME_inpoly.message);
    end

    % If loop and inside check complete without finding masking condition, is_masked remains false.

end % END OF FUNCTION wireframe_OML_masking


% =========================================================================
% INTERNAL HELPER: 2D Line Segment Intersection Check
% =========================================================================
function intersects = line_segment_intersection_2d(p1, p2, p3, p4)
    % Checks if two 2D line segments [p1,p2] and [p3,p4] intersect.
    % Based on algorithm by Gavin Crabb, StackOverflow. Robust to collinear cases.
    % Inputs: p1, p2, p3, p4 are [1x2] vectors [x, y].

    intersects = false; % Default assumption

    % Calculate vectors for the segments
    r = p2 - p1; % Vector for segment 1
    s = p4 - p3; % Vector for segment 2

    % Calculate denominator for intersection parameters: r x s (cross product in 2D)
    r_cross_s = r(1)*s(2) - r(2)*s(1);

    % Calculate vector from start of segment 1 to start of segment 2
    q_minus_p = p3 - p1;

    % --- Check if segments are collinear ---
    % If r_cross_s is close to zero, lines are parallel or collinear.
    if abs(r_cross_s) < 1e-9
        % Calculate cross product (q - p) x r. If zero, they are collinear.
        q_minus_p_cross_r = q_minus_p(1)*r(2) - q_minus_p(2)*r(1);
        if abs(q_minus_p_cross_r) < 1e-9
            % --- Segments are collinear: Check for overlap ---
            r_dot_r = dot(r, r);
            s_dot_r = dot(s, r); % Projection factor of s onto r

            if r_dot_r < 1e-9 % Check if segment 1 has zero length (p1=p2)
                 % Intersection only if p1 lies on segment [p3, p4]
                 if dot(q_minus_p, q_minus_p) < 1e-9 % Check if p1 == p3
                     intersects = true; return;
                 end
                 % Check if p1 is within the bounds of segment [p3, p4] (more complex check needed)
                 return; % Assume no intersection for zero-length segment 1 for simplicity here
            end

            % Calculate parameters t0 and t1 for the projection of segment 2 onto line 1
            t0 = dot(q_minus_p, r) / r_dot_r; % Projection of p3 start relative to p1 start
            t1 = t0 + s_dot_r / r_dot_r;      % Projection of p4 end relative to p1 start

            % Check for overlap between interval [0, 1] (segment 1) and [t0, t1] (segment 2 projection)
            % Allow small tolerance for floating point comparisons
            if max(0, min(t0, t1)) <= min(1, max(t0, t1)) + 1e-9 % Standard overlap condition: max(start) <= min(end)
                 intersects = true;
            end
        end
        % If parallel but not collinear, they don't intersect.
        return; % Exit function
    end

    % --- Segments are not parallel: Calculate intersection parameters t and u ---
    % t: parameter along segment 1 where intersection occurs (0 <= t <= 1)
    % u: parameter along segment 2 where intersection occurs (0 <= u <= 1)
    t = (q_minus_p(1)*s(2) - q_minus_p(2)*s(1)) / r_cross_s;
    u = (q_minus_p(1)*r(2) - q_minus_p(2)*r(1)) / r_cross_s;

    % Check if the intersection point lies within BOTH line segments (parameters between 0 and 1).
    % Allow small tolerance (e.g., 1e-9) for floating point comparisons at endpoints.
    if (t >= -1e-9 && t <= 1.0 + 1e-9 && u >= -1e-9 && u <= 1.0 + 1e-9)
        intersects = true;
    end

end % END line_segment_intersection_2d