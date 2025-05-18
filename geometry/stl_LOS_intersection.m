% =========================================================================
% STL LINE-OF-SIGHT (LOS) INTERSECTION (FINAL - Re-Commented)
% Version: FINAL 1.0
%
% Description:
%   Checks if a line segment (representing Line-of-Sight) intersects with
%   a 3D aircraft model defined by an STL file. It uses the efficient
%   Möller–Trumbore ray-triangle intersection algorithm. This provides
%   accurate, geometry-based aircraft self-masking checks.
%
% Critical Input Requirement: Coordinate Frame Consistency!
%   - This function assumes that the input transmitter position (`tx_pos_local`),
%     receiver position (`rx_pos_local`), and the vertex data within the
%     `stl_filepath` are all defined in the **SAME local coordinate system**,
%     typically the aircraft's body-fixed frame relative to its center of gravity (CG).
%   - The calling function (usually `rf_propagation_model.m`) is responsible
%     for performing the necessary coordinate transformations to get the Tx and Rx
%     positions into this local frame before calling this function.
%
% Inputs:
%   tx_pos_local - Transmitter position [X, Y, Z] in the STL's local coordinate frame (meters).
%                  This is usually the antenna's offset relative to the aircraft CG.
%   rx_pos_local - Receiver position [X, Y, Z] transformed into the STL's local coordinate frame (meters).
%   stl_filepath - Full path string to the STL file (.stl) defining the aircraft geometry.
%   varargin     - Optional inputs (currently unused). Could potentially pass aircraft state
%                  if the STL model itself needed to be rotated/translated (less common).
%
% Output:
%   is_masked    - Boolean: True if the LOS segment (Tx to Rx) intersects any
%                  triangle in the STL model, False otherwise (LOS is clear).
%
% Dependencies:
%   - Requires the `stlread` function to load STL files. This function is
%     available in some MATLAB toolboxes (like Robotics System Toolbox) or
%     can be obtained from MATLAB File Exchange. Ensure it's available on your path.
%
% Caching:
%   - Includes a basic persistent cache (`stl_cache`) to avoid repeatedly
%     reading and parsing the same STL file within a simulation run, improving performance.
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-04]
% =========================================================================
function is_masked = stl_LOS_intersection(tx_pos_local, rx_pos_local, stl_filepath, varargin)

    % Default output: Assume LOS is not masked
    is_masked = false;

    % --- Input Validation ---
    % Check if Tx/Rx positions are valid 3-element numeric vectors
    if ~isnumeric(tx_pos_local) || ~isvector(tx_pos_local) || numel(tx_pos_local) ~= 3 || ...
       ~isnumeric(rx_pos_local) || ~isvector(rx_pos_local) || numel(rx_pos_local) ~= 3
        warning('[STL MASK] Invalid local Tx/Rx position inputs (must be 3-element numeric vectors). Assuming no masking.');
        return;
    end
    % Check if filepath is a valid string
     if ~ischar(stl_filepath) && ~isstring(stl_filepath) || isempty(stl_filepath)
         warning('[STL MASK] STL filepath must be a non-empty string. Assuming no masking.');
         return;
     end
    % Check if the STL file actually exists
    if ~isfile(stl_filepath)
        warning('[STL MASK] STL file not found: %s. Assuming no masking.', stl_filepath);
        return; % Cannot proceed without the file
    end

    % --- Load/Cache STL Data ---
    % Use a persistent variable to cache loaded STL data. `containers.Map` is suitable.
    persistent stl_cache;
    if isempty(stl_cache)
        stl_cache = containers.Map('KeyType', 'char', 'ValueType', 'any');
        fprintf('Debug [STL MASK]: Initialized STL cache.\n');
    end

    % Check cache first
    if stl_cache.isKey(stl_filepath)
        stl_data = stl_cache(stl_filepath); % Load from cache
        % fprintf('Debug [STL MASK]: Loaded %s from cache.\n', stl_filepath); % Optional debug message
    else
        % If not in cache, load from file
        try
            fprintf('Debug [STL MASK]: Loading STL file: %s\n', stl_filepath); % Log loading attempt
            % Call stlread - ensure this function is available on MATLAB path!
            stl_struct = stlread(stl_filepath);

            % Extract vertices and faces (handle different naming conventions from stlread versions)
            if isfield(stl_struct, 'vertices') && isfield(stl_struct, 'faces')
                vertices = stl_struct.vertices; faces = stl_struct.faces;
            elseif isfield(stl_struct, 'Points') && isfield(stl_struct, 'ConnectivityList')
                vertices = stl_struct.Points; faces = stl_struct.ConnectivityList;
            else
                 error('Unrecognized STL structure format from stlread. Check function output.');
            end

            % Basic validation of loaded data
            if isempty(vertices) || isempty(faces) || size(vertices,2) ~= 3 || size(faces,2) ~= 3
                error('STL file loaded but contains invalid or empty vertices/faces data.');
            end

            % Store valid data in cache
            stl_data.vertices = vertices;
            stl_data.faces    = faces;
            stl_cache(stl_filepath) = stl_data;
            fprintf('Debug [STL MASK]: Loaded and cached STL data for %s (%d vertices, %d faces).\n', stl_filepath, size(vertices,1), size(faces,1));

        catch ME_stlread % Handle errors during file reading or parsing
            warning('[STL MASK] Error reading/parsing STL file "%s": %s. Assuming no masking.', stl_filepath, ME_stlread.message);
            % Store an indicator in cache to prevent retrying problematic file? Optional.
            % stl_cache(stl_filepath) = struct('error', ME_stlread.message);
            return; % Cannot proceed if STL load failed
        end
    end

    % Check if cached data indicates a previous error
    if isfield(stl_data, 'error'), return; end

    % Extract vertices and faces from cached/loaded data
    vertices = stl_data.vertices; % N x 3 matrix of vertex coordinates
    faces    = stl_data.faces;    % M x 3 matrix of face vertex indices

    % --- Prepare Ray for Intersection Check ---
    % Ensure positions are row vectors [1x3] for consistency
    ray_origin = tx_pos_local(:)';
    ray_vector = rx_pos_local(:)' - ray_origin; % Vector from Tx to Rx

    % Calculate the length of the LOS segment
    ray_length = norm(ray_vector);

    % Check for zero length (Tx and Rx at the same local point)
    if ray_length < 1e-9 % Use a small tolerance
        % Points coincide. LOS is technically clear unless inside geometry?
        % For masking, if Tx/Rx are same point, assume not masked by surface.
        return; % is_masked remains false
    end

    % Normalize the direction vector (required by Möller–Trumbore)
    ray_direction = ray_vector / ray_length;

    % --- Perform Ray-Triangle Intersection Test for Each Face ---
    num_faces = size(faces, 1);
    % Optional performance logging (can be slow for large models)
    % fprintf('Debug [STL MASK]: Checking intersection against %d faces...\n', num_faces);
    % tic_intersect = tic;

    for i = 1:num_faces
        try
            % Get triangle vertex coordinates using indices from the faces matrix
            v1 = vertices(faces(i, 1), :);
            v2 = vertices(faces(i, 2), :);
            v3 = vertices(faces(i, 3), :);
        catch ME_index % Catch indexing errors (e.g., if face indices are out of bounds)
            warning('[STL MASK] Invalid vertex index in face %d (File: %s). Skipping face. Error: %s', i, stl_filepath, ME_index.message);
            continue; % Skip this face
        end

        % Call the Möller–Trumbore intersection algorithm (internal helper function)
        [intersect, t] = ray_triangle_intersection_mt(ray_origin, ray_direction, v1, v2, v3);

        % --- Check if Intersection Occurred AND is Within the Segment ---
        if intersect
            % 't' is the distance along the ray direction to the intersection point.
            % Check if 't' is within the bounds [0, ray_length] (allow small tolerance).
            if t >= -1e-9 && t <= ray_length + 1e-9
                % Intersection found within the line segment from Tx to Rx.
                is_masked = true;
                % Optional debug message:
                % fprintf('Debug [STL MASK]: Intersection found with face %d at distance t=%.4f (Segment length=%.4f).\n', i, t, ray_length);
                % toc_intersect = toc(tic_intersect);
                % fprintf('Debug [STL MASK]: Intersection found after %.4f sec.\n', toc_intersect);
                return; % Exit loop and function early, LOS is blocked
            end
        end
    end % End loop through faces

    % toc_intersect = toc(tic_intersect);
    % fprintf('Debug [STL MASK]: Finished checking %d faces in %.4f sec. No intersection found.\n', num_faces, toc_intersect);

    % If the loop completes without finding a valid intersection within the segment, LOS is clear.
    % is_masked remains false (the default initialized value).

end % END OF FUNCTION stl_LOS_intersection


% =========================================================================
% INTERNAL HELPER: Möller–Trumbore Ray-Triangle Intersection Algorithm
% =========================================================================
function [intersects, t, u, v] = ray_triangle_intersection_mt(orig, dir, vert0, vert1, vert2)
    % Efficiently checks if a ray intersects with a triangle in 3D space.
    % Inputs:
    %   orig  - Ray origin point [1x3] (meters)
    %   dir   - Ray direction vector [1x3] (MUST be normalized)
    %   vert0, vert1, vert2 - Triangle vertices [1x3] each (meters)
    % Outputs:
    %   intersects - Boolean: True if the ray intersects the triangle (within bounds).
    %   t          - Distance along the ray direction to the intersection point.
    %                Valid only if intersects is true.
    %   u, v       - Barycentric coordinates of intersection (valid if intersects=true).

    % Initialize outputs
    intersects = false;
    t = Inf; u = 0; v = 0;
    EPSILON = 1e-9; % Small tolerance for floating point comparisons

    % Calculate triangle edge vectors sharing vert0
    edge1 = vert1 - vert0; % Vector from vert0 to vert1
    edge2 = vert2 - vert0; % Vector from vert0 to vert2

    % --- Begin MT Algorithm ---
    % Calculate determinant component using cross product: h = dir x edge2
    h = cross(dir, edge2, 2); % Use cross product along dimension 2 (row vector)

    % Calculate determinant: a = edge1 · h
    a = dot(edge1, h, 2); % Use dot product along dimension 2

    % Check if ray is parallel to the triangle plane (determinant is close to zero)
    if abs(a) < EPSILON
        return; % No intersection possible if parallel
    end

    % Calculate inverse determinant
    f = 1.0 / a;

    % Calculate vector from vert0 to ray origin: s = orig - vert0
    s = orig - vert0;

    % Calculate barycentric coordinate 'u': u = f * (s · h)
    u = f * dot(s, h, 2);

    % Check 'u' boundary condition: Is intersection outside edge defined by edge1?
    if u < -EPSILON || u > 1.0 + EPSILON % Allow small tolerance for edges
        return; % Intersection is outside the triangle
    end

    % Calculate vector q = s x edge1
    q = cross(s, edge1, 2);

    % Calculate barycentric coordinate 'v': v = f * (dir · q)
    v = f * dot(dir, q, 2);

    % Check 'v' and 'u+v' boundary conditions: Is intersection outside edge defined by edge2 or outside triangle?
    if v < -EPSILON || (u + v) > 1.0 + EPSILON % Allow small tolerance
        return; % Intersection is outside the triangle
    end

    % Calculate distance 't' along the ray to the intersection point: t = f * (edge2 · q)
    t = f * dot(edge2, q, 2);

    % Check if intersection point is along the positive direction of the ray.
    % For LOS segment checks, we generally only care about intersections *in front*
    % of the origin (t >= 0), but allow a small tolerance in case Tx is exactly on surface.
    if t >= -EPSILON
        intersects = true; % Intersection found within triangle bounds and along ray path
    end
    % If t < 0, the intersection is behind the ray origin.
end % END ray_triangle_intersection_mt