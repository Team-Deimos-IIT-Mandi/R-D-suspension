%% stair_descent_kc_sweep.m
% =========================================================================
% (k,c) SWEEP on a staircase descent, 3 s per run.
%   - step height 200 mm, rover speed 0.5 m/s, same input on both wheels
%   - TIRE (TPU spring #1) stiffness reduced to 20000 N/m
%   - main strut k and c swept over the same ranges as before:
%         k = 2000 ... 15000 N/m,   c = 50 ... 600 N*s/m,   10 x 10 = 100 runs
%   - every run lasts 3 s: 1 s flat (rover settles under gravity, baseline
%     angle measured), then the stairs start (edges at 1.0, 1.6, 2.2, 2.8 s)
%   - only the CHASSIS ANGLE is analysed and plotted
%
% USE WITH: simulation_actualrover_FIXED.slx
% GRAVITY MUST BE ON in the model (Mechanism Configuration, [0 0 -9.81]).
% =========================================================================
clear; clc; close all;

%% ======================= 1. USER SETTINGS ================================
mdl = 'simulation_actualrover_FIXED';
useParallel = false;           % true needs Parallel Computing Toolbox

% ---- suspension ----
k_tpu = 20000;  c_tpu = 500;   % TIRE: stiffness reduced to 20000 N/m (damping unchanged)
k_up  = 50000;  c_up  = 500;   % spring #2 (wheel block <-> rocker): kept at the PREVIOUS values - EDIT to your real values
L0_tpu = 0.15;                 % m  TPU spring natural length

% ---- sweep (same ranges as before) ----
kRange = [2000 15000];         % N/m
cRange = [50   600];           % N*s/m
nFull   = 10;                  % grid points per axis -> 10 x 10 = 100 runs
nSample = 3;                   % trace plot shows nSample x nSample of those runs

% ---- simulation ----
T_end   = 3;                   % s   duration of EACH run
maxStep = 0.005;               % s
T_flat  = 1.0;                 % s   flat ground before the first edge (gravity sag settles, baseline measured)

% ---- staircase ----
stair.h        = 0.200;        % m     step height
stair.v        = 0.5;          % m/s   rover speed (unchanged)
stair.tread    = 0.30;         % m     tread length - ASSUMED, EDIT to your stair
stair.dropTime = pi*sqrt(stair.h/(2*9.81));   % s  edge drop time (~0.32 s, ground never falls faster than gravity)
stair.t0       = T_flat;       % s     first edge
stair.nSteps   = Inf;
stair.dt       = 0.001;
stair.frame    = 'rover';      % 'rover' = bounded sawtooth in a frame descending with the stairs (use if the
                               %           chassis/base is fixed to the world); 'world' = raw staircase

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

%% ======================= 3. BUILD THE STAIRCASE ==========================
Tstep = stair.tread / stair.v;
if stair.dropTime >= Tstep
    error('Edge drop time (%.2f s) must be shorter than the step time (%.2f s).', stair.dropTime, Tstep);
end
t = (0:stair.dt:T_end)';
s = t - stair.t0;
h = zeros(size(t));
on = s >= 0;
j = floor(s(on) / Tstep);
r = s(on) - j*Tstep;
frac = ones(size(r));
ie = r < stair.dropTime;
frac(ie) = 0.5*(1 - cos(pi*r(ie)/stair.dropTime));
n = min(j + frac, stair.nSteps);
world = -stair.h * n;
if strcmp(stair.frame, 'rover')
    sEff = min(s(on), stair.nSteps*Tstep);
    h(on) = world + (stair.h/Tstep) * sEff;
else
    h(on) = world;
end
nEdges = min(floor((T_end - stair.t0)/Tstep) + 1, stair.nSteps);
edgeT  = stair.t0 + (0:nEdges-1)*Tstep;

terrain = timeseries(h, t);                    % same ground height for BOTH wheels
fprintf('Staircase: %.0f mm steps, %.2f s per step, edges at t = %s s (last one is cut off at %g s).\n', ...
    stair.h*1000, Tstep, mat2str(edgeT, 3), T_end);
fprintf('Ground input range (frame = %s): %.0f mm to %.0f mm.\n', stair.frame, min(h)*1000, max(h)*1000);

cfg.T_end = T_end;  cfg.maxStep = maxStep;  cfg.T_flat = T_flat;
cfg.k_tpu = k_tpu;  cfg.c_tpu = c_tpu;  cfg.k_up = k_up;  cfg.c_up = c_up;
cfg.L0_tpu = L0_tpu;  cfg.chunk = 8;  if useParallel, cfg.chunk = 32; end

%% ======================= 4. RUN THE SWEEP ================================
kVals = linspace(kRange(1), kRange(2), nFull);
cVals = linspace(cRange(1), cRange(2), nFull);
[KK, CC] = meshgrid(kVals, cVals);             % rows = c, cols = k
fprintf('\nSweep: %d x %d = %d runs of %g s, tire k = %g N/m ...\n', nFull, nFull, numel(KK), T_end, k_tpu);
res = local_run_batch(mdl, cfg, terrain, KK(:)', CC(:)', useParallel, edgeT, Tstep);

maxGrid = reshape([res.maxDeg], size(KK));
minGrid = reshape([res.minDeg], size(KK));
p2pGrid = reshape([res.p2p],    size(KK));

nFail = sum(~cellfun(@isempty, {res.err}));
nRest = sum([res.rest] > 0.05);
tpuAll = reshape([res.tpuMin], 2, []);
nBott = sum(min(tpuAll, [], 1) < 0.03);
fprintf('Done. %d runs failed to execute, %d runs were not settled before the first edge (> 0.05 deg), %d runs bottomed out the TPU spring (< 3 cm).\n', ...
    nFail, nRest, nBott);
if nFail > 0
    warning('First failure message: %s', res(find(~cellfun(@isempty, {res.err}), 1)).err);
end

% ---- best combinations (smallest peak-to-peak chassis angle) ----
valid = find(isfinite([res.p2p]));
[~, ord] = sort([res(valid).p2p]);
top = valid(ord(1:min(5, numel(ord))));
fprintf('\nTop %d combinations (smallest peak-to-peak chassis angle):\n', numel(top));
fprintf('     k (N/m)   c (N*s/m)   max (deg)   min (deg)   p2p (deg)   first-edge peak (deg)\n');
for q = top
    fprintf('  %9.0f  %10.0f  %10.2f  %10.2f  %10.2f  %12.2f\n', KK(q), CC(q), res(q).maxDeg, res(q).minDeg, res(q).p2p, res(q).firstPeak);
end

%% ======================= 5. PLOTS (chassis angle only) ===================
% ---- maps of the chassis angle over (k,c) ----
figure('Name', 'Chassis angle vs (k,c) - stair descent', 'Position', [80 80 1300 450]);
tl = tiledlayout(1,3,'TileSpacing','compact','Padding','compact');
nexttile; imagesc(kVals, cVals, maxGrid); set(gca,'YDir','normal'); colorbar;
xlabel('k (N/m)'); ylabel('c (N\cdot s/m)'); title('Max chassis angle change (deg)');
nexttile; imagesc(kVals, cVals, minGrid); set(gca,'YDir','normal'); colorbar;
xlabel('k (N/m)'); ylabel('c (N\cdot s/m)'); title('Min chassis angle change (deg)');
nexttile; imagesc(kVals, cVals, p2pGrid); set(gca,'YDir','normal'); colorbar;
xlabel('k (N/m)'); ylabel('c (N\cdot s/m)'); title('Peak-to-peak chassis angle (deg)');
title(tl, sprintf('Stair descent, %.0f mm steps at %.1f m/s, tire k = %g N/m', stair.h*1000, stair.v, k_tpu));

% ---- traces of a nSample x nSample subset of the grid ----
ik = round(linspace(1, nFull, nSample));       % k indices (columns)
ic = round(linspace(1, nFull, nSample));       % c indices (rows)
yAll = [];
for a = 1:nSample
    for b = 1:nSample
        yAll = [yAll; res(sub2ind(size(KK), ic(a), ik(b))).trace_dev(:)]; %#ok<AGROW>
    end
end
yl = [min(yAll) max(yAll)];  if ~all(isfinite(yl)), yl = [-30 10]; end
yl = yl + 0.05*diff(yl)*[-1 1];

fig = figure('Name', 'Chassis angle traces - stair descent', 'Position', [100 60 1250 800]);
t2 = tiledlayout(fig, nSample, nSample, 'TileSpacing','compact','Padding','compact');
for a = 1:nSample                              % rows: c from small to large
    for b = 1:nSample                          % columns: k from small to large
        q  = sub2ind(size(KK), ic(a), ik(b));
        ax = nexttile(t2);
        if ~isempty(res(q).err) || isempty(res(q).trace_t)
            title(ax, sprintf('k=%.0f, c=%.0f [FAILED]', KK(q), CC(q)), 'FontSize', 8);  continue;
        end
        plot(ax, res(q).trace_t, res(q).trace_dev, 'LineWidth', 1); hold(ax, 'on'); grid(ax, 'on');
        for e = 1:numel(edgeT), xline(ax, edgeT(e), ':r'); end
        xlim(ax, [0 T_end]);  ylim(ax, yl);
        title(ax, sprintf('k=%.0f, c=%.0f\nmax %.1f, min %.1f, p2p %.1f deg', KK(q), CC(q), ...
            res(q).maxDeg, res(q).minDeg, res(q).p2p), 'FontSize', 8);
        xlabel(ax, 'Time (s)');  ylabel(ax, '\Delta angle (deg)');
    end
end
title(t2, 'Chassis angle change (red dotted = start of each stair edge)');
drawnow;

%% ========================================================================
%%                          LOCAL FUNCTIONS
%% ========================================================================

function m = local_empty_metrics()
m = struct('maxDeg', NaN, 'minDeg', NaN, 'p2p', NaN, 'firstPeak', NaN, 'rest', NaN, ...
    'tpuMin', [NaN NaN], 'trace_t', [], 'trace_dev', [], 'err', '');
end

function m = local_metrics(out, cfg, edgeT, Tstep)
% Angles are relative to the chassis angle just before the first edge (last 0.4 s of the flat period).
m = local_empty_metrics();
[ta, aa] = local_get_signal(out, 'link3_angle');  aa = aa(:,1);
[tu, iu] = unique(ta);
tq = (0:0.001:cfg.T_end)';
aq = interp1(tu, aa(iu), tq, 'linear', 'extrap');

iB = tq >= cfg.T_flat-0.4 & tq < cfg.T_flat;
a0 = mean(aq(iB));
m.rest = rad2deg(abs(mean(aq(tq >= cfg.T_flat-0.4 & tq < cfg.T_flat-0.2)) - mean(aq(tq >= cfg.T_flat-0.2 & tq < cfg.T_flat))));
dev = rad2deg(aq - a0);

w = tq >= cfg.T_flat;
m.maxDeg = max(dev(w));
m.minDeg = min(dev(w));
m.p2p    = m.maxDeg - m.minDeg;
w1 = tq >= edgeT(1) & tq < edgeT(1) + Tstep;
m.firstPeak = max(abs(dev(w1)));

m.trace_t   = tq(1:10:end);
m.trace_dev = dev(1:10:end);
try
    for side = 1:2
        [~, gp] = local_get_signal(out, sprintf('tpu_gap%d', side));
        m.tpuMin(side) = min(gp(:,1)) * 0.01;          % cm -> m
    end
catch
    m.tpuMin = [NaN NaN];
end
end

function m = local_safe_metrics(out, cfg, edgeT, Tstep)
m = local_empty_metrics();
try
    if ~isempty(out.ErrorMessage), m.err = out.ErrorMessage; return; end
    m = local_metrics(out, cfg, edgeT, Tstep);
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

function in = local_make_input(mdl, cfg, terrain, k, c)
in = Simulink.SimulationInput(mdl);
in = in.setVariable('k', k);              in = in.setVariable('c', c);
in = in.setVariable('k_tpu', cfg.k_tpu);  in = in.setVariable('c_tpu', cfg.c_tpu);
in = in.setVariable('k_up',  cfg.k_up);   in = in.setVariable('c_up',  cfg.c_up);
in = in.setVariable('L0_tpu1', cfg.L0_tpu);
in = in.setVariable('L0_tpu2', cfg.L0_tpu);
in = in.setVariable('terrain1', terrain);   % same ground for left ...
in = in.setVariable('terrain2', terrain);   % ... and right wheel
in = in.setModelParameter('StopTime', num2str(cfg.T_end), 'MaxStep', num2str(cfg.maxStep), ...
    'SimscapeLogType', 'none');
end

function res = local_run_batch(mdl, cfg, terrain, kList, cList, useParallel, edgeT, Tstep)
N = numel(kList);
res = repmat(local_empty_metrics(), 1, N);
t0 = tic;
for s = 1:cfg.chunk:N
    idx = s:min(s + cfg.chunk - 1, N);
    clear in;
    for q = 1:numel(idx)
        in(q) = local_make_input(mdl, cfg, terrain, kList(idx(q)), cList(idx(q)));   %#ok<AGROW>
    end
    try
        if useParallel
            out = parsim(in, 'ShowProgress', 'off', 'TransferBaseWorkspaceVariables', 'on', 'UseFastRestart', 'off');
        else
            out = sim(in, 'ShowProgress', 'off', 'UseFastRestart', 'off');
        end
        for q = 1:numel(idx)
            res(idx(q)) = local_safe_metrics(out(q), cfg, edgeT, Tstep);
        end
    catch ME
        for q = 1:numel(idx), res(idx(q)).err = ME.message; end
    end
    fprintf('  %d / %d runs done (%.1f min elapsed)\n', idx(end), N, toc(t0)/60);
end
end
