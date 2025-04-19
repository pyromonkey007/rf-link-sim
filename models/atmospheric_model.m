% =========================================================================
% ATMOSPHERIC MODEL (FINAL - Re-Commented)
% Version: FINAL 1.0
%
% Description:
%   Provides atmospheric properties (density, temperature, pressure) as a
%   function of altitude above Mean Sea Level (MSL). Supports multiple
%   standard atmospheric profiles (ISA, hot, cold, etc.) selectable via
%   configuration. Includes functions for basic True Airspeed (TAS) to
%   Calibrated Airspeed (KCAS) conversions (approximations).
%
% Usage:
%   - Initialized once in `main_simulation.m`:
%     `atmosphere = atmospheric_model(config.environment.MODEL);`
%   - Methods called within simulation loop (e.g., `aircraft_dynamics_model`):
%     `atmos_cond = atmosphere.get_conditions(altitude_m_msl);`
%     `tas_mps = atmosphere.kcas_to_tas(kcas, alt_m);`
%     `kcas = atmosphere.tas_to_kcas(tas_mps, alt_m);` % Placeholder requires implementation
%
% Models Implemented (selected via `config.environment.MODEL`):
%   - 'standard': International Standard Atmosphere (ISA).
%   - 'hot': ISA conditions + 15°C at sea level (example).
%   - 'cold': ISA conditions - 15°C at sea level (example).
%   - 'humid': Uses standard Temp/Pressure for calculations here; humidity
%              parameter mainly for potential use in advanced RF loss models.
%   - 'conservative': Example using ISA + 5°C (often used for performance margins).
%
% Constants Used (Internal):
%   - Standard Gravity (g0)
%   - Specific Gas Constant for Dry Air (R_specific)
%   - Sea Level Pressure (P0), Temperature (T0) - Model dependent
%   - Temperature Lapse Rate (L) - Model dependent
%   - Tropopause Altitude (H_tropopause) - Model dependent
%
% Placeholders:
%   - `tas_to_kcas_local`: Needs proper implementation considering compressibility.
%   - Humidity effect on density/RF loss is not fully implemented here.
%   - Assumes simple troposphere + isothermal stratosphere model.
%
% Maintainer: [Your Name/Team]
% Last Updated: [Date - e.g., 2025-04-04]
% =========================================================================
function atmosphere_obj = atmospheric_model(model_type)
    % Input: model_type - String identifier for the desired atmospheric profile.

    fprintf('[ATMOS] Initializing Atmospheric Model: %s\n', model_type);

    % Internal structure to hold model constants and parameters
    atmos = struct();

    % --- Define Standard Physical Constants ---
    atmos.g0          = 9.80665;        % Standard gravity (m/s^2)
    atmos.R_specific  = 287.058;        % Specific gas constant for dry air (J/(kg·K))
    % Define base ISA parameters for reference
    T0_isa        = 288.15;         % ISA Sea Level Temp (K) = 15°C
    P0_isa        = 101325;         % ISA Sea Level Pressure (Pa)
    L_isa         = 0.0065;         % ISA Temp Lapse Rate in Troposphere (K/m)
    H_tropo_isa   = 11000;          % ISA Tropopause Altitude (m)
    rho0_isa      = 1.225;          % ISA Sea Level Density (kg/m^3) - For TAS/EAS conversions

    % --- Select Model Parameters based on `model_type` ---
    switch lower(model_type)
        case 'standard' % International Standard Atmosphere (ISA)
            T0 = T0_isa; P0 = P0_isa; L = L_isa; H_tropo = H_tropo_isa;
        case 'hot'      % Example Hot Day: ISA + 15°C
            T0 = T0_isa + 15; P0 = P0_isa; L = L_isa; H_tropo = H_tropo_isa; % Assume standard pressure/lapse/tropo
        case 'cold'     % Example Cold Day: ISA - 15°C
            T0 = T0_isa - 15; P0 = P0_isa; L = L_isa; H_tropo = H_tropo_isa; % Assume standard pressure/lapse/tropo
        case 'humid'    % Placeholder: Uses Standard T/P for calculations here.
            T0 = T0_isa; P0 = P0_isa; L = L_isa; H_tropo = H_tropo_isa;
            atmos.relative_humidity_percent = 70; % Store humidity value (e.g., for external use in RF loss)
        case 'conservative' % Example Conservative: ISA + 5°C
            T0 = T0_isa + 5; P0 = P0_isa; L = L_isa; H_tropo = H_tropo_isa; % Assume standard pressure/lapse/tropo
        otherwise
            warning('Atmospheric model type "%s" not recognized. Defaulting to "standard".', model_type);
            model_type = 'standard'; % Use standard as fallback
            T0 = T0_isa; P0 = P0_isa; L = L_isa; H_tropo = H_tropo_isa;
    end

    % Store selected parameters in the internal structure
    atmos.T0           = T0;                % Sea Level Temperature (K)
    atmos.P0           = P0;                % Sea Level Pressure (Pa)
    atmos.L            = L;                 % Temperature Lapse Rate (K/m)
    atmos.H_tropopause = H_tropo;           % Tropopause Altitude (m)
    atmos.T_tropopause = atmos.T0 - atmos.L * atmos.H_tropopause; % Temp at Tropopause (K)
    atmos.model_type   = lower(model_type); % Store the selected model name
    atmos.rho0_isa     = rho0_isa;          % Store ISA sea level density for conversions

    fprintf('  Using %s parameters: T0=%.2f K (%.1f C), P0=%.0f Pa, L=%.4f K/m\n', ...
            model_type, atmos.T0, atmos.T0 - 273.15, atmos.P0, atmos.L);

    % --- Assign Public Function Handles to the Output Object ---
    % These handles allow calling the internal calculation functions using dot notation (e.g., atmosphere.get_conditions(alt)).
    atmosphere_obj.get_conditions = @(alt_m) get_conditions_local(atmos, alt_m);
    % Convenience functions:
    atmosphere_obj.get_density    = @(alt_m) get_conditions_local(atmos, alt_m).density_kg_m3;
    atmosphere_obj.get_temperature= @(alt_m) get_conditions_local(atmos, alt_m).temperature_k;
    atmosphere_obj.get_pressure   = @(alt_m) get_conditions_local(atmos, alt_m).pressure_pa;
    % Airspeed conversions:
    atmosphere_obj.kcas_to_tas    = @(kcas, alt_m) kcas_to_tas_local(atmos, kcas, alt_m);
    atmosphere_obj.tas_to_kcas    = @(tas_mps, alt_m) tas_to_kcas_local(atmos, tas_mps, alt_m); % Placeholder!
    % Expose internal parameters (read-only) if needed externally
    atmosphere_obj.params         = atmos;

    fprintf('[ATMOS] Atmospheric model initialized successfully.\n');

end % END OF FUNCTION atmospheric_model


% =========================================================================
% INTERNAL CALCULATION FUNCTIONS (Nested or Local)
% These functions use the 'atmos' structure from the parent scope.
% =========================================================================

% -------------------------------------------------------------------------
% GET ATMOSPHERIC CONDITIONS AT ALTITUDE
% -------------------------------------------------------------------------
function conditions = get_conditions_local(atmos, altitude_m_msl)
    % Calculates temperature, pressure, and density at a given altitude.
    % Inputs:
    %   atmos          - Internal structure containing model parameters.
    %   altitude_m_msl - Altitude above Mean Sea Level (meters).
    % Output:
    %   conditions     - Structure containing: .temperature_k, .pressure_pa, .density_kg_m3

    % Initialize outputs
    temperature_k = NaN; pressure_pa = NaN; density_kg_m3 = NaN;

    try
        % Ensure altitude is not negative for calculations.
        h = max(altitude_m_msl, 0);

        % --- Temperature Calculation ---
        % Uses linear lapse rate up to the tropopause, then assumes isothermal above.
        if h <= atmos.H_tropopause
            % Below or at the tropopause
            temperature_k = atmos.T0 - atmos.L * h;
        else
            % Above the tropopause (isothermal layer in this simple model)
            temperature_k = atmos.T_tropopause;
        end
        % Enforce physical lower limit (absolute zero)
        temperature_k = max(temperature_k, 0.1);

        % --- Pressure Calculation ---
        % Uses the appropriate form of the barometric formula.
        if h <= atmos.H_tropopause
            if abs(atmos.L) < 1e-9 % Handle isothermal case (L=0) to avoid division by zero
                 pressure_pa = atmos.P0 * exp(-atmos.g0 * h / (atmos.R_specific * atmos.T0));
            else % Standard lapse rate formula
                 pressure_pa = atmos.P0 * (1 - atmos.L * h / atmos.T0)^(atmos.g0 / (atmos.R_specific * atmos.L));
            end
        else % Above tropopause
            % Calculate pressure at tropopause first
             P_tropopause = atmos.P0 * (1 - atmos.L * atmos.H_tropopause / atmos.T0)^(atmos.g0 / (atmos.R_specific * atmos.L));
             % Then apply isothermal formula from tropopause upwards
             pressure_pa = P_tropopause * exp(-atmos.g0 * (h - atmos.H_tropopause) / (atmos.R_specific * atmos.T_tropopause));
        end
        % Enforce physical lower limit (near vacuum)
        pressure_pa = max(pressure_pa, 1e-3);

        % --- Density Calculation ---
        % Uses the Ideal Gas Law: rho = P / (R_specific * T)
        % --- Humidity Impact on Air Density ---
        if strcmp(atmos.model_type, 'humid')
            % Saturation Vapor Pressure (approximation)
            es = 6.11 * 10.0^(7.5*(temperature_k - 273.15)/(temperature_k - 35.85)); % in mb
            % Actual Vapor Pressure (assume relative humidity)
            e = es * atmos.relative_humidity_percent / 100;
            
            % Calculate the mass of water vapor per unit volume
            R_vapor = 461.5; % Specific gas constant for water vapor (J/(kg·K))
            rho_vapor = e / (R_vapor * temperature_k); % in kg/m^3
            
            % Adjust total air density by adding vapor density
            density_kg_m3 = (pressure_pa / (atmos.R_specific * temperature_k)) + rho_vapor;
        else
            % --- Density Calculation for Non-Humid Air ---
            density_kg_m3 = pressure_pa / (atmos.R_specific * temperature_k);
        end

        % Enforce physical lower limit
        density_kg_m3 = max(density_kg_m3, 1e-5);

    catch ME_cond % Catch any errors during calculation
        warning('Error calculating atmospheric conditions at %.1f m: %s. Returning NaNs.', altitude_m_msl, ME_cond.message);
        temperature_k = NaN; pressure_pa = NaN; density_kg_m3 = NaN;
    end

    % Return results in a structure
    conditions = struct(...
        'temperature_k', temperature_k, ...
        'pressure_pa',   pressure_pa, ...
        'density_kg_m3', density_kg_m3 ...
    );
end

% -------------------------------------------------------------------------
% KCAS TO TAS CONVERSION (Approximation)
% -------------------------------------------------------------------------
function tas_mps = kcas_to_tas_local(atmos, kcas, alt_m)
    % Converts Knots Calibrated Air Speed (KCAS) to True Air Speed (TAS) in m/s.
    % Approximation: Assumes KCAS ~ EAS (Equivalent Airspeed). This holds reasonably
    % well at lower speeds and altitudes but ignores compressibility effects.
    % TAS = EAS / sqrt(rho/rho0)
    KTS2MPS = 0.514444; % Knots to m/s

    conditions = get_conditions_local(atmos, alt_m); % Get density at altitude
    rho        = conditions.density_kg_m3;
    rho0_isa   = atmos.rho0_isa; % ISA sea level density

    if rho <= 0 || isnan(rho)
        warning('Atmospheric density invalid (%.3f) at %.1f m. Cannot calculate TAS.', rho, alt_m);
        tas_mps = NaN;
        return;
    end

    % Approximate KCAS as EAS (Equivalent Airspeed)
    eas_mps = kcas * KTS2MPS;

    % Convert EAS to TAS using the density ratio
    tas_mps = eas_mps / sqrt(rho / rho0_isa);
end

% -------------------------------------------------------------------------
% TAS TO KCAS CONVERSION (Placeholder)
% -------------------------------------------------------------------------
function kcas = tas_to_kcas_local(atmos, tas_mps, alt_m)
    % Converts True Air Speed (TAS) in m/s to Knots Calibrated Air Speed (KCAS).
    % ** PLACEHOLDER - NEEDS PROPER IMPLEMENTATION **
    % This requires inverting the KCAS -> TAS relationship, ideally including
    % compressibility effects (Mach number dependency).
    MPS2KTS = 1 / 0.514444; % m/s to Knots

    conditions = get_conditions_local(atmos, alt_m); % Get density at altitude
    rho        = conditions.density_kg_m3;
    rho0_isa   = atmos.rho0_isa;

    if rho <= 0 || isnan(rho)
        warning('Atmospheric density invalid (%.3f) at %.1f m. Cannot calculate KCAS.', rho, alt_m);
        kcas = NaN;
        return;
    end

    % Simple inversion of the approximate EAS = TAS * sqrt(rho/rho0) formula
    eas_mps = tas_mps * sqrt(rho / rho0_isa);

    % Approximate EAS as KCAS
    kcas = eas_mps * MPS2KTS;

    warning('tas_to_kcas calculation uses simple density ratio approximation. Implement compressibility effects for higher fidelity.');
    % ** END PLACEHOLDER **
end