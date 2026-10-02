%% spring_design_from_sweep.m
% =========================================================================
% Takes the best FEASIBLE designs from suspension_sweep_results.csv
% (written by suspension_design_sweep.m) and designs a coil spring for each.
%
% Values taken from the sweep, per design:
%   k_target = k                         strut stiffness found by the sweep
%   F_max    = forceMax_N * forceMargin  peak strut force (needs the strut_force log)
%   stroke   = F_max / k_actual          deflection from free length to peak load
%                                        (so preload and sag are already included)
%   travel   = strutMax_m - strutMin_m   total strut travel, checked against the damper stroke
%
% If forceMax_N is missing (no strut_force log), the script falls back to
% F_max = k * travel. That leaves out preload and static sag, so it UNDERESTIMATES
% the real load - log strut_force before trusting the result.
% strutMax_m is the largest damper length; the 20-25 cm limit is checked in the sweep.
%
% Needs spring_design.m on the path.
% =========================================================================
clear; clc;

%% ---- USER SETTINGS ----
csvFile      = 'suspension_sweep_results.csv';
nDesigns     = 3;        % how many of the best feasible designs to process
damperOD     = 0.028;    % m   outer diameter of the Misumi damper body the spring wraps around (EDIT, [] if beside it)
damperStroke = 0.025;    % m   stroke of the chosen damper (EDIT)
forceMargin  = 1.2;      % factor on the peak force from the sweep
rankBy       = 'p2p';    % column of the CSV used to rank feasible designs

%% ---- READ THE SWEEP RESULTS ----
T = readtable(csvFile);
Tf = T(T.feasible == 1, :);
if isempty(Tf)
    error('No feasible design in %s. Fix the constraints or the model before designing the spring.', csvFile);
end
Tf = sortrows(Tf, rankBy);
nDesigns = min(nDesigns, height(Tf));

hasForce = ismember('forceMax_N', Tf.Properties.VariableNames) && any(~isnan(Tf.forceMax_N));
if ~hasForce
    warning('No forceMax_N in the CSV: F_max falls back to k * travel (no preload or sag).');
end

%% ---- DESIGN A SPRING FOR EACH ----
results = cell(1, nDesigns);
for n = 1:nDesigns
    r = Tf(n, :);
    travel = r.strutMax_m - r.strutMin_m;                   % m
    if hasForce && ~isnan(r.forceMax_N)
        F = r.forceMax_N * forceMargin;
    else
        F = r.k * travel * forceMargin;
    end
    delta = F / r.k;                                        % free length to peak load

    fprintf('\n=== Design %d: k = %.0f N/m, c = %.0f N*s/m (tire %.0f N/m) ===\n', n, r.k, r.c, r.k_tpu);
    fprintf('F_max = %.0f N, deflection to peak = %.1f mm, strut travel = %.1f mm (damper stroke %.0f mm)\n', ...
        F, 1000*delta, 1000*travel, 1000*damperStroke);
    if ~isnan(travel) && travel > damperStroke
        fprintf('WARNING: strut travel exceeds the damper stroke - the damper would bottom out in this design.\n');
    end
    results{n} = spring_design(r.k, F, delta, damperOD);
end
