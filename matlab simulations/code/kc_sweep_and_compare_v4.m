%% kc_sweep_and_compare_v4.m   (terrain-driven, 150 s run, 3 m/s)
% =========================================================================
% CHANGES vs v3
%   * Pebbles are 20-50 mm high: base height in 5 mm steps (20,25,...,50)
%     plus a random +/-5 mm scatter, hard-clipped to 20-50 mm.
%   * Rover speed 3 m/s (pebbles are lengthened so they last 17-50 ms).
%   * PASS / FAIL uses ONLY the chassis angle on the pebble terrain, with
%     outliers removed (the top ter.outlierPct % of |angle| samples are
%     ignored). Target: robust peak chassis angle inside 5-10 deg.
%   * The 190 mm obstacles are OFF by default. If you switch them on
%     (ter.includeObstacles = true) they are only REPORTED, never used for
%     pass/fail.
%   * Only chassis-angle graphs are drawn (plus one optional sweep map of
%     that same angle). Terrain / 0 mm plots are switched off by default.
%   * Wider default k,c range (the old 6750-6850 range showed no k effect).
%   * Same run count as before: 10x10 grid + 3x3 samples.
% =========================================================================
clear; clc; close all;

%% ======================= 1. USER SETTINGS ================================
mdl = 'simulation_actualrover_FIXED';   % model name (file name without .slx)

% ---- switches ----
applyModelEdits         = false;
saveModelAfterEdits     = false;
matchStrutToInitialPose = true;
tpuSensorsRewired       = true;
runGeometryCheck        = true;
runPartA                = true;
runPartB                = true;
useParallel             = false;
confirmLongRuns         = true;

% ---- plots (chassis angle only) ----
plotTerrain   = false;   % true = also draw the ground height under each wheel
plotCheck     = false;   % true = also draw the 0 mm check
showSweepMap  = true;    % one figure: robust peak chassis angle vs (k,c) + pass map

% ---- simulation ----
T_end   = 30;     % s
maxStep = 0.002;   % s   small step: pebbles are fast at 3 m/s

% ---- rover geometry constants (metres) ----
g.rockerX       = 0.345;
g.rockerEndY    = 0.235;
g.rockerAttachZ = -0.105;
g.L_up          = 0.22;
g.rimH          = 0.05;
g.L_tpu         = 0.15;
g.L0_strut      = 0.2528;

% ---- suspension parameters ----
k_tpu = 50000;   c_tpu = 500;
k_up  = k_tpu;   c_up  = c_tpu;      % placeholders - EDIT to your real values
kRange = [2000 15000];               % N/m     main strut stiffness swept
cRange = [50   600];                 % N*s/m   main strut damping swept
nFull   = 10;                        % Part A grid points per axis (10x10 = 100 runs)
nSample = 3;                         % Part B: nSample x nSample combinations
k_fixed = mean(kRange);  c_fixed = mean(cRange);

% ---- target: judged on PEBBLE terrain only, outliers removed ----
targetPeakMinDeg = 5;      % deg  robust peak chassis angle should be >= this ...
targetPeakMaxDeg = 10;     % deg  ... and <= this   (set Min = 0 if you only want "<= 10")
outlierPct       = 0.5;    % %    the largest 0.5 % of |angle| samples are treated as outliers and ignored

% ---- terrain ("road") settings ----
ter.T_end       = T_end;
ter.dt          = 0.0005;          % s      terrain sample time
ter.v           = 3;               % m/s    rover speed
ter.T_flat      = 10;              % s      0 mm at start (rover settles, baseline measured)
ter.pebH        = [0.020 0.050];   % m      pebble height limits 20-50 mm
ter.pebStep     = 0.005;           % m      base heights in 5 mm steps
ter.pebScatter  = 0.005;           % m      random +/- scatter added to each pebble
ter.pebLen      = [0.05 0.15];     % m      pebble length along ground (17-50 ms at 3 m/s)
ter.pebMeanGap  = 1.5;             % s      average EXTRA flat time between pebbles
ter.pebMinFlat  = 0.3;             % s      guaranteed flat time between pebbles
ter.includeObstacles = false;      % true = add the two 190 mm obstacles (reported only, NOT used for pass/fail)
ter.hObs        = 0.190;
ter.tObs        = [50 110];
if ~ter.includeObstacles, ter.tObs = []; end
ter.obsRiseLen  = 0.20;
ter.obsTopLen   = 0.30;
ter.obsOnWheel  = [true true];     % [left right]  e.g. [true false] = one-sided hit
ter.preFlat     = 5;
ter.postFlat    = 10;
ter.seed        = 1;               % identical road in every run

% ---- metric settings ----
cfg.T_end = T_end;  cfg.maxStep = maxStep;
cfg.T_flat = ter.T_flat;  cfg.preFlat = ter.preFlat;  cfg.postFlat = ter.postFlat;
cfg.recoverWin     = ter.postFlat - 1;
cfg.settleBandFrac = 0.05;
cfg.settleBandMin  = deg2rad(0.05);
cfg.outlierPct     = outlierPct;
cfg.k_tpu = k_tpu; cfg.c_tpu = c_tpu; cfg.k_up = k_up; cfg.c_up = c_up;
cfg.L0_tpu1 = g.L_tpu; cfg.L0_tpu2 = g.L_tpu;
cfg.tpuSensors = tpuSensorsRewired;
cfg.chunk      = 8;  if useParallel, cfg.chunk = 32; end
cfg.keepTraces = false;

pkLo = deg2rad(targetPeakMinDeg);  pkHi = deg2rad(targetPeakMaxDeg);

%% ======================= 2. LOAD MODEL + APPLY GEOMETRY ==================
load_system(mdl);
if useParallel, saveModelAfterEdits = true; end
if applyModelEdits
    fprintf('Applying geometry numbers to the model...\n');
    local_apply_model_edits(mdl, g, matchStrutToInitialPose);
    if saveModelAfterEdits, save_system(mdl); fprintf('Model saved.\n'); end
end
local_preflight(mdl);
local_check_geometry(mdl, g, matchStrutToInitialPose);

%% ======================= 3. BUILD THE ROAD ===============================
terr = local_build_terrain(ter);
terrain1 = timeseries(terr.hL, terr.t);   terrain1.Name = 'terrain1';
terrain2 = timeseries(terr.hR, terr.t);   terrain2.Name = 'terrain2';

fprintf('Road built: %d pebbles left, %d pebbles right (heights %.0f-%.0f mm), %d obstacles.\n', ...
    size(terr.pebL,1), size(terr.pebR,1), 1000*min([terr.pebL(:,3); terr.pebR(:,3)]), ...
    1000*max([terr.pebL(:,3); terr.pebR(:,3)]), numel(terr.obsStart));

if plotTerrain
    figure('Name','Terrain input (ground height under each wheel)');
    subplot(2,1,1); plot(terr.t, terr.hL*1000,'b'); grid on; ylabel('Left ground (mm)');
    title('Ground height under the wheels');
    subplot(2,1,2); plot(terr.t, terr.hR*1000,'r'); grid on; ylabel('Right ground (mm)'); xlabel('Time (s)');
    drawnow;
end

%% ======================= 4. 0 mm CHECK: does the rover stay put? =========
tCheck = NaN;  Tchk = 8;
if runGeometryCheck
    fprintf('\n0 mm check: %g s of flat ground...\n', Tchk);
    cfgChk = cfg;  cfgChk.T_end = Tchk;
    inChk = local_make_input(mdl, cfgChk, k_fixed, c_fixed);
    zeroTs = timeseries([0;0], [0;Tchk]);
    inChk = inChk.setVariable('terrain1', zeroTs);
    inChk = inChk.setVariable('terrain2', zeroTs);
    tic;
    try
        outChk = sim(inChk);
        tCheck = toc;
    catch ME
        error('0 mm check failed to run:\n%s\nFix this before running the sweeps.', ME.message);
    end
    if ~isempty(outChk.ErrorMessage), error('0 mm check failed:\n%s', outChk.ErrorMessage); end

    [tc, ac] = local_get_signal(outChk, 'link3_angle');  ac = ac(:,1);
    [tu, iu] = unique(tc);  tq = (0:0.01:Tchk)';  aq = interp1(tu, ac(iu), tq, 'linear', 'extrap');
    moveDeg = rad2deg(max(abs(aq - aq(1))));
    lateDeg = rad2deg(max(abs(aq(tq > Tchk-2) - aq(end))));
    fprintf('  chassis angle moved at most %.3f deg from its initial value (still moving in last 2 s: %.4f deg)\n', moveDeg, lateDeg);
    if moveDeg < 1
        fprintf('  OK: the rover stays in its initial configuration at 0 mm input.\n');
    else
        warning(['The rover does NOT stay put at 0 mm input (moved %.2f deg). Likely causes: ' ...
            '(1) Spring 1/2 natural length is not the assembled length -> matchStrutToInitialPose = true; ' ...
            '(2) a guide joint is not vertical; (3) TPU gap wrong.'], moveDeg);
    end
    if lateDeg > 0.05
        warning('Still moving at the end of the 0 mm check -> raise ter.T_flat or check damping.');
    end
    if tpuSensorsRewired
        try
            for side = 1:2
                [~, gp] = local_get_signal(outChk, sprintf('tpu_gap%d', side));
                gp = gp(:,1) * 0.01;
                fprintf('  TPU spring %d length: start %.4f m, end %.4f m (natural %.3f m)\n', side, gp(1), gp(end), g.L_tpu);
                if abs(gp(1) - g.L_tpu) > 0.01
                    warning('TPU spring %d starts %.3f m long, expected %.3f m.', side, gp(1), g.L_tpu);
                end
            end
        catch ME
            warning('Could not read tpu_gap1/2 (%s). Setting tpuSensorsRewired = false.', ME.message);
            cfg.tpuSensors = false;
        end
    end
    if plotCheck
        figure('Name','0 mm check (chassis angle)');
        plot(tq, rad2deg(aq - aq(1))); grid on; xlabel('Time (s)'); ylabel('Chassis angle change (deg)');
        title('0 mm input: chassis angle should barely move'); drawnow;
    end
end

%% ======================= 5. RUN-TIME ESTIMATE ============================
nRuns = 0;  if runPartA, nRuns = nRuns + nFull^2; end;  if runPartB, nRuns = nRuns + nSample^2; end
if ~isnan(tCheck) && nRuns > 0
    perRun = tCheck * (T_end / Tchk);
    est = perRun * nRuns;
    fprintf('\nEstimated sweep time: about %.1f min per run x %d runs = %.1f h (upper estimate).\n', perRun/60, nRuns, est/3600);
    if confirmLongRuns && est > 2*3600
        ansr = input('That is long. Continue? (y/n) ', 's');
        if ~strcmpi(ansr, 'y'), fprintf('Stopped. Lower nFull (or set runPartA=false) and rerun.\n'); return; end
    end
end

%% ======================= 6. PART A: full (k,c) grid ======================
if runPartA
    kVals = linspace(kRange(1), kRange(2), nFull);
    cVals = linspace(cRange(1), cRange(2), nFull);
    [KK, CC] = meshgrid(kVals, cVals);            % rows = c, cols = k
    fprintf('\nPart A: %d x %d = %d runs...\n', nFull, nFull, numel(KK));
    resA = local_run_batch(mdl, cfg, terr, KK(:)', CC(:)', useParallel, 'Part A');

    peakGrid = reshape([resA.peakRob], size(KK));     % rad, outliers removed
    rmsGrid  = reshape([resA.rmsPeb],  size(KK));     % rad
    okGrid   = isfinite(peakGrid) & isfinite(rmsGrid);
    passGrid = okGrid & (peakGrid >= pkLo) & (peakGrid <= pkHi);
    nFail = sum(~cellfun(@isempty, {resA.err}));
    fprintf('Part A done. %d of %d combinations have a robust peak chassis angle in %g-%g deg. %d runs failed to execute.\n', ...
        sum(passGrid(:)), numel(KK), targetPeakMinDeg, targetPeakMaxDeg, nFail);
    if nFail > 0
        firstErr = resA(find(~cellfun(@isempty, {resA.err}), 1)).err;
        warning('First failure message: %s', firstErr);
    end

    if showSweepMap
        figure('Name','Part A: robust peak chassis angle vs (k,c)');
        tl = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
        nexttile; imagesc(kVals, cVals, rad2deg(peakGrid)); set(gca,'YDir','normal'); colorbar;
        xlabel('k (N/m)'); ylabel('c (N\cdot s/m)');
        title(sprintf('Peak chassis angle, outliers removed (deg) - target %g-%g', targetPeakMinDeg, targetPeakMaxDeg));
        nexttile; imagesc(kVals, cVals, double(passGrid)); set(gca,'YDir','normal'); colorbar;
        xlabel('k (N/m)'); ylabel('c (N\cdot s/m)'); title('1 = peak inside target band');
        title(tl, 'Full (k,c) sweep - pebble terrain'); drawnow;
    end
end

%% ======================= 7. PART B: chassis-angle traces =================
if runPartB
    kS = linspace(kRange(1), kRange(2), nSample);
    cS = linspace(cRange(1), cRange(2), nSample);
    [KS, CS] = meshgrid(kS, cS);   KS = KS'; CS = CS';
    nCombos = numel(KS);
    fprintf('\nPart B: %d representative combinations...\n', nCombos);
    cfgB = cfg;  cfgB.keepTraces = true;
    resB = local_run_batch(mdl, cfgB, terr, KS(:)', CS(:)', useParallel, 'Part B');

    peakList = [resB.peakRob];  rmsList = [resB.rmsPeb];  rawList = [resB.peakRaw];
    okList   = isfinite(peakList) & isfinite(rmsList);
    passList = okList & (peakList >= pkLo) & (peakList <= pkHi);

    fig4 = figure('Name','Part B: chassis angle change, full 150 s');
    t1 = tiledlayout(fig4, nSample, nSample, 'TileSpacing','compact','Padding','compact');
    fig5 = figure('Name','Part B: chassis angle, zoom on 20 s of pebbles');
    t2 = tiledlayout(fig5, nSample, nSample, 'TileSpacing','compact','Padding','compact');
    zoomWin = [ter.T_flat, ter.T_flat + 20];
    for idx = 1:nCombos
        st = "FAIL";  if passList(idx), st = "PASS"; end
        ttl = sprintf('k=%.0f, c=%.0f [%s]\npeak=%.2f deg (raw %.2f), RMS=%.3f deg', KS(idx), CS(idx), st, ...
            rad2deg(peakList(idx)), rad2deg(rawList(idx)), rad2deg(rmsList(idx)));
        for f = 1:2
            if f == 1, ax = nexttile(t1); else, ax = nexttile(t2); end
            r = resB(idx);
            if ~isempty(r.err) || isempty(r.trace_t)
                title(ax, sprintf('k=%.0f, c=%.0f\n[FAILED]', KS(idx), CS(idx)), 'FontSize', 8);  continue;
            end
            plot(ax, r.trace_t, rad2deg(r.trace_dev), 'LineWidth', 1); hold(ax, 'on'); grid(ax, 'on');
            yline(ax,  targetPeakMaxDeg, ':m');  yline(ax, -targetPeakMaxDeg, ':m');
            yline(ax,  targetPeakMinDeg, ':g');  yline(ax, -targetPeakMinDeg, ':g');
            for j = 1:numel(terr.obsStart)
                xline(ax, terr.obsStart(j), ':r');  xline(ax, terr.obsEnd(j), ':r');
            end
            if f == 2, xlim(ax, zoomWin); end
            title(ax, ttl, 'FontSize', 8);  xlabel(ax, 'Time (s)');  ylabel(ax, '\Delta angle (deg)');
        end
    end
    title(t1, 'Chassis angle change from 0 mm rest angle (green dotted = 5 deg, magenta dotted = 10 deg)');
    title(t2, 'Zoom: 20 s of pebble terrain');

    if any(okList)
        mid = 0.5*(pkLo + pkHi);
        cand = find(passList);  header = 'Best combination inside the target band (lowest RMS)';
        if ~isempty(cand)
            [~, ib] = min(rmsList(cand));  best = cand(ib);
        else
            cand = find(okList);  header = 'No sampled combination inside the band - closest one';
            [~, ib] = min(abs(peakList(cand) - mid));  best = cand(ib);
        end
        fprintf('\n%s (of %d sampled):\n  k = %.1f N/m, c = %.1f N*s/m\n', header, nCombos, KS(best), CS(best));
        fprintf('  robust peak = %.2f deg (raw peak incl. outliers %.2f deg), RMS = %.3f deg\n', ...
            rad2deg(peakList(best)), rad2deg(rawList(best)), rad2deg(rmsList(best)));
        for j = 1:numel(terr.obsStart)
            fprintf('    obstacle %d (t=%g s, reported only): peak %.2f deg, settle %.2f s\n', j, terr.obsStart(j), ...
                rad2deg(resB(best).peak(j)), resB(best).ts(j));
        end
        if cfg.tpuSensors && all(isfinite(resB(best).tpuMin))
            fprintf('  smallest TPU spring length: left %.3f m, right %.3f m (natural %.2f m)\n', ...
                resB(best).tpuMin(1), resB(best).tpuMin(2), g.L_tpu);
            if min(resB(best).tpuMin) < 0.03
                warning('TPU spring almost fully compressed -> the wheel "bottoms out"; results are not trustworthy.');
            end
        end
    else
        fprintf('\nAll sampled combinations failed to execute or log data.\n');
    end
end

%% ========================================================================
%%                          LOCAL FUNCTIONS
%% ========================================================================

function terr = local_build_terrain(p)
% Ground height [m] under each wheel: 0 mm flat + random pebbles (+ optional obstacles).
t = (0:p.dt:p.T_end)';
n = numel(t);
Tr   = p.obsRiseLen / p.v;
Tt   = p.obsTopLen  / p.v;
Tobs = 2*Tr + Tt;
obsStart = p.tObs(:)';
obsEnd   = obsStart + Tobs;
if any(obsStart - p.preFlat < p.T_flat), error('First obstacle starts too early: need tObs >= T_flat + preFlat.'); end
if any(obsEnd + p.postFlat > p.T_end),   error('Last obstacle ends too late: need tObs + duration + postFlat <= T_end.'); end
if numel(obsStart) > 1 && any(diff(obsStart) < Tobs + p.postFlat + p.preFlat), error('Obstacles overlap their flat guard zones.'); end
excl = [obsStart' - p.preFlat, obsEnd' + p.postFlat];

hObs = zeros(n,1);
for j = 1:numel(obsStart)
    s = t - obsStart(j);
    iu = s >= 0        & s < Tr;          hObs(iu) = p.hObs * 0.5*(1 - cos(pi*s(iu)/Tr));
    it = s >= Tr       & s < Tr + Tt;     hObs(it) = p.hObs;
    id = s >= Tr + Tt  & s <= Tobs;       hObs(id) = p.hObs * 0.5*(1 + cos(pi*(s(id) - Tr - Tt)/Tr));
end

rng(p.seed);
[pebL, hL] = local_pebbles(t, p, excl);
[pebR, hR] = local_pebbles(t, p, excl);
hL = hL + double(p.obsOnWheel(1)) * hObs;
hR = hR + double(p.obsOnWheel(2)) * hObs;

terr = struct('t', t, 'hL', hL, 'hR', hR, 'pebL', pebL, 'pebR', pebR, ...
    'obsStart', obsStart, 'obsEnd', obsEnd, 'excl', excl);
end

function [peb, h] = local_pebbles(t, p, excl)
% Random pebbles: smooth half-wave bumps [start width height].
% Height = random 5 mm-step base (20..50 mm) + random +/- scatter, clipped to the 20-50 mm limits.
h   = zeros(size(t));
peb = zeros(0,3);
tc  = p.T_flat;
nSteps = round((p.pebH(2)-p.pebH(1)) / p.pebStep);
while true
    tc = tc + p.pebMinFlat - p.pebMeanGap * log(1 - rand());
    w  = (p.pebLen(1) + (p.pebLen(2)-p.pebLen(1))*rand()) / p.v;
    Hb = p.pebH(1) + p.pebStep * randi([0 nSteps]);
    H  = Hb + p.pebScatter * (2*rand() - 1);
    H  = min(max(H, p.pebH(1)), p.pebH(2));
    if tc + w > p.T_end - 1, break; end
    hit = find(tc < excl(:,2) & (tc + w) > excl(:,1), 1);
    if ~isempty(hit)
        tc = excl(hit,2);
        continue;
    end
    idx = t >= tc & t <= tc + w;
    h(idx) = H * 0.5 * (1 - cos(2*pi*(t(idx) - tc)/w));
    peb(end+1,:) = [tc w H];                                  %#ok<AGROW>
    tc = tc + w;
end
end

function m = local_metrics(out, terr, cfg)
% All angles are relative to the chassis angle at 0 mm input (last 2 s of the initial flat period).
m = local_empty_metrics(numel(terr.obsStart));
[t, a] = local_get_signal(out, 'link3_angle');  a = a(:,1);
[tu, iu] = unique(t);
tq = (0:0.001:cfg.T_end)';
aq = interp1(tu, a(iu), tq, 'linear', 'extrap');

iB = tq >= cfg.T_flat-2 & tq < cfg.T_flat;
m.a0    = mean(aq(iB));
m.drift = abs(mean(aq(tq >= cfg.T_flat-2 & tq < cfg.T_flat-1)) - mean(aq(tq >= cfg.T_flat-1 & tq < cfg.T_flat)));
dev = aq - m.a0;

% ---- obstacles (REPORTED ONLY, never used for pass/fail) ----
for j = 1:numel(terr.obsStart)
    t0 = terr.obsStart(j);  t1 = terr.obsEnd(j);
    base = mean(aq(tq >= t0-2 & tq < t0-0.2));
    win  = tq >= t0 & tq <= t1 + cfg.recoverWin;
    m.peak(j) = max(abs(aq(win) - base));
    iS   = tq >= t1 & tq <= t1 + cfg.recoverWin;
    fin  = mean(aq(tq > t1 + cfg.recoverWin - 1 & tq <= t1 + cfg.recoverWin));
    band = max(cfg.settleBandFrac * m.peak(j), cfg.settleBandMin);
    outside = abs(aq(iS) - fin) > band;
    tS = tq(iS);
    if ~any(outside),      m.ts(j) = 0;
    elseif outside(end),   m.ts(j) = Inf;
    else,                  m.ts(j) = tS(find(outside, 1, 'last')) - t1;
    end
end
if ~isempty(m.peak), m.peakObs = max(m.peak); m.settleObs = max(m.ts); end

% ---- pebble region: this is what pass/fail is based on ----
mask = tq >= cfg.T_flat;
for j = 1:numel(terr.obsStart)
    mask(tq >= terr.obsStart(j) - cfg.preFlat & tq <= terr.obsEnd(j) + cfg.postFlat) = false;
end
s = sort(abs(dev(mask)));                                     % ascending |angle|
nKeep  = max(1, floor((1 - cfg.outlierPct/100) * numel(s)));  % drop the largest outlierPct % of samples
keep   = s(1:nKeep);
m.peakRaw = s(end);                                           % incl. outliers (for information)
m.peakRob = keep(end);                                        % outliers removed -> used for pass/fail
m.rmsPeb  = sqrt(mean(keep.^2));

if cfg.keepTraces
    m.trace_t   = tq(1:10:end);
    m.trace_dev = dev(1:10:end);
end
if cfg.tpuSensors
    try
        for side = 1:2
            [~, gp] = local_get_signal(out, sprintf('tpu_gap%d', side));
            m.tpuMin(side) = min(gp(:,1)) * 0.01;
        end
    catch
        m.tpuMin = [NaN NaN];
    end
end
end

function m = local_empty_metrics(nObs)
m = struct('peakObs', NaN, 'settleObs', NaN, 'peak', NaN(1,nObs), 'ts', NaN(1,nObs), ...
    'rmsPeb', NaN, 'peakRob', NaN, 'peakRaw', NaN, 'drift', NaN, 'a0', NaN, 'tpuMin', [NaN NaN], ...
    'trace_t', [], 'trace_dev', [], 'err', '');
end

function m = local_safe_metrics(out, terr, cfg)
m = local_empty_metrics(numel(terr.obsStart));
try
    if ~isempty(out.ErrorMessage), m.err = out.ErrorMessage; return; end
    m = local_metrics(out, terr, cfg);
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

function in = local_make_input(mdl, cfg, k, c)
in = Simulink.SimulationInput(mdl);
in = in.setVariable('k', k);              in = in.setVariable('c', c);
in = in.setVariable('k_tpu', cfg.k_tpu);  in = in.setVariable('c_tpu', cfg.c_tpu);
in = in.setVariable('k_up',  cfg.k_up);   in = in.setVariable('c_up',  cfg.c_up);
in = in.setVariable('L0_tpu1', cfg.L0_tpu1);
in = in.setVariable('L0_tpu2', cfg.L0_tpu2);
in = in.setModelParameter('StopTime', num2str(cfg.T_end), 'MaxStep', num2str(cfg.maxStep), ...
    'SimscapeLogType', 'none');
end

function res = local_run_batch(mdl, cfg, terr, kList, cList, useParallel, label)
N = numel(kList);
res = repmat(local_empty_metrics(numel(terr.obsStart)), 1, N);
t0 = tic;
for s = 1:cfg.chunk:N
    idx = s:min(s + cfg.chunk - 1, N);
    clear in;
    for q = 1:numel(idx)
        in(q) = local_make_input(mdl, cfg, kList(idx(q)), cList(idx(q)));   %#ok<AGROW>
    end
    try
        if useParallel
            out = parsim(in, 'ShowProgress', 'off', 'TransferBaseWorkspaceVariables', 'on', 'UseFastRestart', 'off');
        else
            out = sim(in, 'ShowProgress', 'off', 'UseFastRestart', 'off');
        end
        for q = 1:numel(idx)
            res(idx(q)) = local_safe_metrics(out(q), terr, cfg);
        end
    catch ME
        for q = 1:numel(idx), res(idx(q)).err = ME.message; end
    end
    fprintf('  %s: %d / %d runs done (%.1f min elapsed)\n', label, idx(end), N, toc(t0)/60);
end
end

function local_apply_model_edits(mdl, g, matchStrut)
zBase = g.rockerAttachZ - g.L_up - g.rimH;
for sid = [91 131]
    local_set_rt(mdl, sid, 'StandardAxis', '+Y', 90, 'Cartesian', [0 0 g.L_up]);
end
for sid = [141 144]
    local_set_rt(mdl, sid, 'None', '', 0, 'Cartesian', [0 0 -g.L_up]);
end
local_set_rt(mdl, 152, 'None', '', 0, 'Cartesian', [-g.rockerX g.rockerEndY zBase]);
local_set_rt(mdl, 156, 'None', '', 0, 'Cartesian', [ g.rockerX g.rockerEndY zBase]);
for sid = [151 155]
    local_set_rt(mdl, sid, 'None', '', 0, 'Cartesian', [0 0 -g.L_tpu]);
end
for sid = [138 142]
    h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, sid));
    set_param(h, 'NaturalLength', num2str(g.L_up), 'NaturalLengthUnits', 'm', ...
        'SpringStiffness', 'k_up', 'DampingCoefficient', 'c_up');
end
for sid = [137 145]
    h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, sid));
    set_param(h, 'BrickDimensions', mat2str([10 10 g.rimH*100]), 'BrickDimensionsUnits', 'cm');
end
if matchStrut
    for sid = [3 24]
        h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, sid));
        set_param(h, 'NaturalLength', num2str(g.L0_strut, '%.5f'), 'NaturalLengthUnits', 'm');
    end
end
set_param(Simulink.ID.getHandle(sprintf('%s:%d', mdl, 177)), 'VariableName', 'terrain1');
set_param(Simulink.ID.getHandle(sprintf('%s:%d', mdl, 176)), 'VariableName', 'terrain2');
for sid = [98 132]
    h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, sid));
    set_param(h, 'SimscapeFilterOrder', '2');
end
fprintf('  geometry edits applied (guide frame z=%.3f, ground joint base z=%.3f, sphere z=%.3f m).\n', ...
    g.rockerAttachZ - g.L_up, zBase, zBase - g.L_tpu);
end

function local_set_rt(mdl, sid, rotMethod, rotAxis, rotDeg, transMethod, offset)
h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, sid));
set_param(h, 'RotationMethod', rotMethod);
if strcmp(rotMethod, 'StandardAxis')
    set_param(h, 'RotationStandardAxis', rotAxis, 'RotationAngle', num2str(rotDeg), 'RotationAngleUnits', 'deg');
end
set_param(h, 'TranslationMethod', transMethod);
if strcmp(transMethod, 'Cartesian')
    set_param(h, 'TranslationCartesianOffset', mat2str(offset), 'TranslationCartesianOffsetUnits', 'm');
end
end

function local_preflight(mdl)
fw = cellstr(find_system(mdl, 'SearchDepth', 1, 'BlockType', 'FromWorkspace'));
names = {};
for i = 1:numel(fw)
    if ~isempty(fw{i}), names{end+1} = strtrim(get_param(fw{i}, 'VariableName')); end %#ok<AGROW>
end
if ~all(ismember({'terrain1', 'terrain2'}, names))
    error(['The model needs two From Workspace blocks with Variable name  terrain1  and  terrain2. ' ...
        'Found: %s'], strjoin(names, ', '));
end
old = [find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Step'); ...
       find_system(mdl, 'SearchDepth', 1, 'BlockType', 'UniformRandomNumber')];
if ~isempty(old)
    warning('The old Step / Uniform Random Number blocks are still in the model (%d found). Delete them.', numel(old));
end
end

function local_check_geometry(mdl, g, matchStrut)
chk = {138, g.L_up,     'spring #2 (rocker <-> wheel block) natural length';
        3,  g.L0_strut, 'main strut (Spring 1) natural length'};
if ~matchStrut, chk(2,:) = []; end
for i = 1:size(chk,1)
    h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, chk{i,1}));
    v = str2double(get_param(h, 'NaturalLength'));
    switch get_param(h, 'NaturalLengthUnits')
        case 'cm', v = v/100;
        case 'mm', v = v/1000;
        case 'm'
        otherwise, v = NaN;
    end
    if isfinite(v) && abs(v - chk{i,2}) > 1e-4
        warning(['The model has %s = %.4f m but this script expects %.4f m. ' ...
            'Set applyModelEdits = true or edit the block by hand.'], chk{i,3}, v, chk{i,2});
    end
end
end
