%% suspension_design_sweep.m
% =========================================================================
% Constrained (k,c) design sweep for the rover suspension.
% Built from stair_descent_kc_sweep.m - same model interface, same helpers.
%
% BOUNDARY CONDITIONS CHECKED ON EVERY RUN
%   1. tire (TPU spring) deformation   <= 10 mm      (L0_tpu - min(tpu_gap))
%   2. damper length                   <= 25 cm      (needs 'strut_len' log, see below)
%   3. wheel lift-off (TPU spring stretched past L0) <= 1 % of the run
%   4. rover speed                     =  3 m/s
%   5. strut c taken ONLY from the Misumi list (fill cMisumi below)
%   6. strut k: 5-digit values, log-spaced (10 000 ... 99 999 N/m)
%
% USE WITH: the model with a FREE chassis and rocker (see notes at the end).
% Signals the model must log:  link3_angle, tpu_gap1, tpu_gap2   (as before)
%                              strut_len  (m, rocker-chassis damper length) - optional
%                              strut_force (N, force in the strut spring-damper block,
%                                          the 'fm' output of Spring 1 / Spring 2) - optional
%   strut_len and strut_force feed spring_design_from_sweep.m through the CSV.
% Workspace variables the model reads: k, c, k_tpu, c_tpu, k_up, c_up,
%                              L0_tpu1, L0_tpu2, terrain1, terrain2,
%                              F_front (N, optional front-wheel force block)
% =========================================================================
clear; clc; close all;

%% ======================= 1. USER SETTINGS ================================
mdl = 'simulation_actualrover_FIXED';   % EDIT: point to the free-chassis version
useParallel = false;                    % true needs Parallel Computing Toolbox

% ---- design speed ----
v = 3.0;                                % m/s

% ---- boundary conditions ----
bc.tireDefMax  = 0.010;                 % m   max tire deformation
bc.strutLenMax = 0.25;                  % m   max damper length (your 20-25 cm window)
bc.liftTol     = 0.001;                 % m   TPU stretched beyond L0 by more than this = wheel off the ground
bc.liftFracMax = 0.01;                  % allowed fraction of time off the ground
bc.angleTarget = 5;                     % deg peak chassis angle target (for the plots)

% ---- fixed suspension parameters ----
L0_tpu  = 0.15;                         % m   TPU spring natural length
k_up    = 50000;  c_up = 500;           % spring #2 (wheel block <-> rocker) - EDIT to the real values
F_front = 0;                            % N   extra front-wheel force (needs a force block reading F_front)
dtWheels = 0;                           % s   terrain2 delay vs terrain1 = wheelbase / v  (EDIT; 0 = both wheels together)

% ---- tire cases to compare, one row each: [k_tpu  c_tpu] ----
tireCases = [20000 500];                % add rows e.g. [20000 500; 30000 500; 30000 1000]

% ---- strut sweep ----
nK    = 8;
kVals = unique(round(logspace(log10(10000), log10(99999), nK)));   % 5-digit, large spread
cMisumi = [];                           % EDIT: Misumi damping coefficients in N*s/m, e.g. [120 250 400 ...]
if isempty(cMisumi)
    warning('cMisumi is empty: using a PLACEHOLDER range 50-600 N*s/m. These are NOT Misumi values.');
    cVals = linspace(50, 600, 6);  cIsMisumi = false;
else
    cVals = sort(cMisumi(:))';      cIsMisumi = true;
end

% ---- simulation ----
T_end = 3;  maxStep = 0.002;  T_flat = 1.0;

% ---- terrain scenario ----
scn.type = 'bump';                      % 'bump' or 'stairs'
scn.v  = v;
scn.t0 = T_flat;                        % first event time
scn.bump  = struct('h', 0.05, 'L', 0.30, 'spacing', 1.0, 'n', 3);  % EDIT: height, length, spacing (m), count
scn.stair = struct('h', 0.200, 'tread', 0.30, 'frame', 'world');   % 'world' for a free chassis
% NOTE: stairs at 3 m/s need tread >= v*dropTime (~0.95 m for 200 mm steps),
%       otherwise the ground would have to fall faster than gravity.

%% ======================= 2. PRE-FLIGHT ===================================
load_system(mdl);
fw = cellstr(find_system(mdl, 'SearchDepth', 1, 'BlockType', 'FromWorkspace'));
names = {};
for i = 1:numel(fw)
    if ~isempty(fw{i}), names{end+1} = strtrim(get_param(fw{i}, 'VariableName')); end %#ok<SAGROW>
end
if ~all(ismember({'terrain1','terrain2'}, names))
    error('The model needs From Workspace blocks with Variable name terrain1 and terrain2. Found: %s', strjoin(names, ', '));
end

%% ======================= 3. TERRAIN ======================================
[terrain1, eventT] = local_profile(scn, T_end, 0);
[terrain2, ~]      = local_profile(scn, T_end, dtWheels);
fprintf('Scenario "%s" at %.1f m/s, events at t = %s s. Ground range %.0f to %.0f mm.\n', ...
    scn.type, v, mat2str(eventT, 3), 1000*min(terrain1.Data), 1000*max(terrain1.Data));

cfg.T_end = T_end;  cfg.maxStep = maxStep;  cfg.T_flat = T_flat;
cfg.k_up = k_up;  cfg.c_up = c_up;  cfg.L0_tpu = L0_tpu;  cfg.F_front = F_front;
cfg.liftTol = bc.liftTol;  cfg.chunk = 8;  if useParallel, cfg.chunk = 32; end

%% ======================= 4. RUN THE SWEEP ================================
[KK, CC] = meshgrid(kVals, cVals);          % rows = c, cols = k
nPer = numel(KK);
nTC  = size(tireCases, 1);
R = cell(1, nTC);
fprintf('\n%d tire case(s) x %d k x %d c = %d runs.\n', nTC, numel(kVals), numel(cVals), nTC*nPer);
for tc = 1:nTC
    cfg.k_tpu = tireCases(tc, 1);  cfg.c_tpu = tireCases(tc, 2);
    fprintf('\nTire case %d: k_tpu = %g N/m, c_tpu = %g N*s/m\n', tc, cfg.k_tpu, cfg.c_tpu);
    R{tc} = local_run_batch(mdl, cfg, terrain1, terrain2, KK(:)', CC(:)', useParallel);
end

%% ======================= 5. RESULTS TABLE ================================
Tall = table();
for tc = 1:nTC
    res = R{tc};  n = numel(res);
    Tt = table(repmat(tireCases(tc,1), n, 1), repmat(tireCases(tc,2), n, 1), KK(:), CC(:), ...
        [res.maxDeg]', [res.minDeg]', [res.p2p]', [res.peakAbs]', [res.rmsDeg]', ...
        1000*[res.tireDef]', [res.liftFrac]', [res.strutMax]', [res.strutMin]', [res.strutStat]', ...
        [res.forceMax]', [res.rest]', ...
        'VariableNames', {'k_tpu','c_tpu','k','c','maxDeg','minDeg','p2p','peakAbs','rmsDeg', ...
                          'tireDef_mm','liftFrac','strutMax_m','strutMin_m','strutStat_m', ...
                          'forceMax_N','rest'});
    Tall = [Tall; Tt]; %#ok<AGROW>
end
Tall.feasible = Tall.tireDef_mm <= 1000*bc.tireDefMax ...
              & Tall.liftFrac   <= bc.liftFracMax ...
              & (isnan(Tall.strutMax_m) | Tall.strutMax_m <= bc.strutLenMax) ...
              & Tall.rest <= 0.05;

nFail = sum(~cellfun(@isempty, cellfun(@(r) {r.err}, R, 'UniformOutput', false)));
fprintf('\nFeasible designs: %d of %d.\n', sum(Tall.feasible), height(Tall));
if ~cIsMisumi, fprintf('WARNING: c values are placeholders, not Misumi values.\n'); end
if all(isnan(Tall.strutMax_m)), fprintf('NOTE: no strut_len signal found - the 25 cm damper limit was NOT checked.\n'); end

if any(Tall.feasible)
    cand = find(Tall.feasible);
    [~, o] = sort(Tall.p2p(cand));
    idxRank = cand(o);
    fprintf('Best feasible designs (smallest peak-to-peak chassis angle):\n');
else
    [~, idxRank] = sort(Tall.tireDef_mm);
    fprintf('NO design meets all constraints. Closest by tire deformation:\n');
end
disp(Tall(idxRank(1:min(8, numel(idxRank))), :));
writetable(Tall, 'suspension_sweep_results.csv');

%% ======================= 6. PLOTS ========================================
for tc = 1:nTC
    res = R{tc};
    G = @(f) reshape([res.(f)], size(KK));
    fig = figure('Name', sprintf('Maps - tire case %d', tc), 'Position', [80 80 1250 800]);
    tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    local_map(nexttile(tl), kVals, cVals, 1000*G('tireDef'), 'Max tire deformation (mm), limit 10');
    local_map(nexttile(tl), kVals, cVals, G('liftFrac'),     'Fraction of time wheel is off the ground');
    local_map(nexttile(tl), kVals, cVals, G('peakAbs'),      sprintf('Peak |chassis angle| (deg), target %g', bc.angleTarget));
    ok = reshape(Tall.feasible((tc-1)*nPer + (1:nPer)), size(KK));
    local_map(nexttile(tl), kVals, cVals, double(ok),        '1 = meets ALL constraints');
    title(tl, sprintf('%s at %.1f m/s, tire k = %g N/m, c = %g N*s/m', scn.type, v, tireCases(tc,1), tireCases(tc,2)));
end

nPlot = min(3, numel(idxRank));
fig = figure('Name', 'Best designs - traces', 'Position', [100 60 1100 700]);
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile(tl); hold(ax1, 'on'); grid(ax1, 'on'); ylabel(ax1, 'Chassis \Delta angle (deg)');
ax2 = nexttile(tl); hold(ax2, 'on'); grid(ax2, 'on'); ylabel(ax2, 'Tire deformation (mm)'); xlabel(ax2, 'Time (s)');
for r = 1:nPlot
    i  = idxRank(r);  tc = ceil(i/nPer);  q = i - (tc-1)*nPer;  rr = R{tc}(q);
    if isempty(rr.trace_t), continue; end
    lbl = sprintf('k=%.0f, c=%.0f (tire %.0f)', Tall.k(i), Tall.c(i), Tall.k_tpu(i));
    plot(ax1, rr.trace_t, rr.trace_dev, 'LineWidth', 1, 'DisplayName', lbl);
    plot(ax2, rr.trace_t, 1000*rr.trace_def, 'LineWidth', 1, 'DisplayName', lbl);
end
yline(ax1, [-bc.angleTarget bc.angleTarget], 'g:');
yline(ax2, 1000*bc.tireDefMax, 'r--');
for e = eventT, xline(ax1, e, ':r'); xline(ax2, e, ':r'); end
legend(ax1, 'Location', 'best');
title(tl, 'Best designs (red dashed = tire deformation limit)');
drawnow;

%% ========================================================================
%%                          LOCAL FUNCTIONS
%% ========================================================================

function [ts, eventT] = local_profile(scn, T_end, delay)
% Ground height timeseries, optionally delayed (for the rear wheel).
dt = 0.001;  t = (0:dt:T_end)';  tt = t - delay;
h = zeros(size(t));
switch scn.type
    case 'bump'
        Tb = scn.bump.L / scn.v;
        eventT = scn.t0 + (0:scn.bump.n-1) * scn.bump.spacing / scn.v;
        for e = eventT
            s = tt - e;  m = s >= 0 & s < Tb;
            h(m) = h(m) + scn.bump.h * 0.5 * (1 - cos(2*pi*s(m)/Tb));
        end
    case 'stairs'
        st = scn.stair;  g = 9.81;
        dropTime = pi*sqrt(st.h/(2*g));
        Tstep = st.tread / scn.v;
        if dropTime >= Tstep
            error(['Stairs: edge drop time (%.2f s) must be shorter than the step time (%.2f s). ' ...
                   'At %.1f m/s the tread must be at least %.2f m.'], dropTime, Tstep, scn.v, scn.v*dropTime);
        end
        s = tt - scn.t0;  on = s >= 0;
        j = floor(s(on) / Tstep);  r = s(on) - j*Tstep;
        frac = ones(size(r));  ie = r < dropTime;
        frac(ie) = 0.5*(1 - cos(pi*r(ie)/dropTime));
        n = j + frac;
        world = -st.h * n;
        if strcmp(st.frame, 'rover')
            h(on) = world + (st.h/Tstep) * s(on);
        else
            h(on) = world;
        end
        nEdges = floor((T_end - scn.t0)/Tstep) + 1;
        eventT = scn.t0 + (0:nEdges-1)*Tstep;
    otherwise
        error('scn.type must be ''bump'' or ''stairs''.');
end
ts = timeseries(h, t);
end

function m = local_empty_metrics()
m = struct('maxDeg', NaN, 'minDeg', NaN, 'p2p', NaN, 'peakAbs', NaN, 'rmsDeg', NaN, 'rest', NaN, ...
    'tireDef', NaN, 'liftFrac', NaN, 'strutMax', NaN, 'strutMin', NaN, 'strutStat', NaN, 'forceMax', NaN, ...
    'trace_t', [], 'trace_dev', [], 'trace_def', [], 'err', '');
end

function y = local_resample(t, y, tq)
[tu, iu] = unique(t(:));
y = y(:);
y = interp1(tu, y(iu), tq, 'linear', 'extrap');
end

function m = local_metrics(out, cfg)
m  = local_empty_metrics();
tq = (0:0.001:cfg.T_end)';

% ---- chassis angle, relative to the settled angle before the first event ----
[ta, aa] = local_get_signal(out, 'link3_angle');
aq = local_resample(ta, aa(:,1), tq);
iB = tq >= cfg.T_flat-0.4 & tq < cfg.T_flat;
a0 = mean(aq(iB));
m.rest = rad2deg(abs(mean(aq(tq >= cfg.T_flat-0.4 & tq < cfg.T_flat-0.2)) - mean(aq(tq >= cfg.T_flat-0.2 & tq < cfg.T_flat))));
dev = rad2deg(aq - a0);
w = tq >= cfg.T_flat;
m.maxDeg  = max(dev(w));  m.minDeg = min(dev(w));
m.p2p     = m.maxDeg - m.minDeg;
m.peakAbs = max(abs(dev(w)));
m.rmsDeg  = sqrt(mean(dev(w).^2));

% ---- tire (TPU spring): deformation and lift-off ----
G = zeros(numel(tq), 2);
for side = 1:2
    [tg, gp] = local_get_signal(out, sprintf('tpu_gap%d', side));
    G(:, side) = local_resample(tg, gp(:,1), tq) * 0.01;      % cm -> m
end
def = cfg.L0_tpu - min(G, [], 2);                             % m, positive = compressed
m.tireDef  = max(def(w));
m.liftFrac = mean(any(G(w,:) > cfg.L0_tpu + cfg.liftTol, 2));

% ---- damper length and strut force (optional signals; one column per side) ----
try
    [ts_, sl] = local_get_signal(out, 'strut_len');
    SL = zeros(numel(tq), size(sl, 2));
    for j = 1:size(sl, 2), SL(:, j) = local_resample(ts_, sl(:, j), tq); end
    m.strutMax  = max(SL(w, :), [], 'all');
    m.strutMin  = min(SL(w, :), [], 'all');
    m.strutStat = mean(SL(iB, :), 'all');       % settled length before the first event
catch
end
try
    [tf_, sf] = local_get_signal(out, 'strut_force');
    SF = zeros(numel(tq), size(sf, 2));
    for j = 1:size(sf, 2), SF(:, j) = local_resample(tf_, sf(:, j), tq); end
    m.forceMax = max(abs(SF(w, :)), [], 'all');
catch
end

m.trace_t = tq(1:10:end);  m.trace_dev = dev(1:10:end);  m.trace_def = def(1:10:end);
end

function m = local_safe_metrics(out, cfg)
m = local_empty_metrics();
try
    if ~isempty(out.ErrorMessage), m.err = out.ErrorMessage; return; end
    m = local_metrics(out, cfg);
catch ME
    m.err = ME.message;
end
end

function [t, y] = local_get_signal(out, name)
v = out.(name);
if isa(v, 'timeseries')
    t = v.Time(:);  y = v.Data;
elseif isstruct(v)
    t = v.time(:);  y = v.signals.values;
else
    t = out.tout(:);  y = v;
end
y = squeeze(y);
if size(y,1) ~= numel(t) && size(y,2) == numel(t), y = y.'; end
end

function in = local_make_input(mdl, cfg, t1, t2, k, c)
in = Simulink.SimulationInput(mdl);
in = in.setVariable('k', k);              in = in.setVariable('c', c);
in = in.setVariable('k_tpu', cfg.k_tpu);  in = in.setVariable('c_tpu', cfg.c_tpu);
in = in.setVariable('k_up',  cfg.k_up);   in = in.setVariable('c_up',  cfg.c_up);
in = in.setVariable('L0_tpu1', cfg.L0_tpu);
in = in.setVariable('L0_tpu2', cfg.L0_tpu);
in = in.setVariable('F_front', cfg.F_front);
in = in.setVariable('terrain1', t1);
in = in.setVariable('terrain2', t2);
in = in.setModelParameter('StopTime', num2str(cfg.T_end), 'MaxStep', num2str(cfg.maxStep), ...
    'SimscapeLogType', 'none');
end

function res = local_run_batch(mdl, cfg, t1, t2, kList, cList, useParallel)
N = numel(kList);
res = repmat(local_empty_metrics(), 1, N);
t0 = tic;
for s = 1:cfg.chunk:N
    idx = s:min(s + cfg.chunk - 1, N);
    clear in;
    for q = 1:numel(idx)
        in(q) = local_make_input(mdl, cfg, t1, t2, kList(idx(q)), cList(idx(q)));   %#ok<AGROW>
    end
    try
        if useParallel
            out = parsim(in, 'ShowProgress', 'off', 'TransferBaseWorkspaceVariables', 'on', 'UseFastRestart', 'off');
        else
            out = sim(in, 'ShowProgress', 'off', 'UseFastRestart', 'off');
        end
        for q = 1:numel(idx)
            res(idx(q)) = local_safe_metrics(out(q), cfg);
        end
    catch ME
        for q = 1:numel(idx), res(idx(q)).err = ME.message; end
    end
    fprintf('  %d / %d runs done (%.1f min elapsed)\n', idx(end), N, toc(t0)/60);
end
end

function local_map(ax, kVals, cVals, Gd, ttl)
imagesc(ax, 1:numel(kVals), 1:numel(cVals), Gd);
set(ax, 'YDir', 'normal');  colorbar(ax);
xticks(ax, 1:numel(kVals));  xticklabels(ax, compose('%d', kVals));  xtickangle(ax, 45);
yticks(ax, 1:numel(cVals));  yticklabels(ax, compose('%g', cVals));
xlabel(ax, 'k (N/m)');  ylabel(ax, 'c (N\cdot s/m)');  title(ax, ttl);
end

%% ========================================================================
%% MODEL CHANGES THIS SCRIPT ASSUMES (do these in the .slx)
%% ========================================================================
% 1. FREE CHASSIS AND ROCKER: the old stair script had a 'rover' frame option
%    meant for a chassis fixed to the world. Replace that fixed base with a
%    6-DOF Joint (or Planar Joint for a 2D model) between World and chassis, keep
%    gravity on, and keep the rocker attached to the chassis only through its
%    pivot and the strut. Then use scn.stair.frame = 'world'.
% 2. LOG strut_len: add a Simscape Spring-Damper / Prismatic Joint position
%    sensor on the rocker-chassis strut, convert to total length (m) and send it
%    to a To Workspace block named strut_len. The 25 cm check only runs if it exists.
% 3. FRONT FORCE (only if lift-off persists with a free chassis): add an External
%    Force and Torque block on each front wheel block, with the force input
%    taken from the workspace variable F_front. Report the F_front used, since
%    it hides lift-off.
% 4. The TPU tire is a two-sided spring: it can stretch to hold the wheel to the
%    ground. liftFrac counts the time it is stretched past L0, which is when a
%    real tire would have left the ground.
