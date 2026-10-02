%% kc_sweep_and_compare.m   (v3 - terrain-driven, 150 s run)
% =========================================================================
% WHAT THIS SCRIPT DOES
%   0. (optional) writes the corrected wheel / ground-input GEOMETRY numbers
%      into the Simulink model (the same numbers as in the step-by-step guide)
%   1. builds ONE "road" (ground height vs time, in metres) for each wheel:
%        - flat 0 mm ground,
%        - random pebbles, 1-10 mm high, at random moments,
%        - exactly TWO 190 mm obstacles (t = 50 s and t = 110 s by default),
%      and hands it to the model through two From Workspace blocks
%      (variables  terrain1 = left wheel,  terrain2 = right wheel)
%   2. runs a short 0 mm-input check: the rover must stay in its initial
%      configuration and the TPU spring must be 15 cm long
%   3. PART A: sweeps main-strut stiffness k and damping c
%      PART B: plots a few (k,c) combinations side by side
%   Pass / fail is judged ONLY on the two 190 mm obstacles:
%        peak chassis angle <= targetAngleDeg   AND   settling time <= targetSettleTime
%   Pebble behaviour is reported (RMS / peak angle) so you can compare designs.
%
% SIGNAL CHAIN IN THE MODEL (what "absorbs energy" where)
%   ground (sphere, moved by the terrain signal)
%     -> TPU spring/damper #1 (k_tpu,c_tpu, 15 cm)  -> wheel block (5 cm rim)
%     -> spring/damper #2 (k_up,c_up, 22 cm)        -> rocker arm
%     -> main strut #3 = Spring 1 / Spring 2 (k,c)  -> bell crank -> chassis
%   Everything is solved simultaneously by Simscape, so every block moves
%   while its springs compress - exactly as on the real rover.
%
% USE WITH:  simulation_actualrover_FIXED.slx  (already contains every model
%   change described in the guide: vertical ground axes, wheel block stacked
%   between the 22 cm and 15 cm springs, From Workspace blocks reading
%   terrain1 / terrain2, Transform Sensors measuring the TPU spring, ...).
%   If you keep working in your OWN model instead, set applyModelEdits = true
%   (numbers) and do the 3 re-wirings of the guide by hand.
% =========================================================================
clear; clc; close all;

%% ======================= 1. USER SETTINGS ================================
mdl = 'simulation_actualrover_FIXED';   % model name (file name without .slx)

% ---- switches ----
applyModelEdits         = false;  % FIXED.slx already has them. true = (re)write the geometry numbers into the model (in memory)
saveModelAfterEdits     = false;  % true = also save them into the .slx (make a backup first!)
matchStrutToInitialPose = true;   % Spring 1/2 natural length := assembled length 25.28 cm (FIXED.slx already has it; otherwise needs applyModelEdits=true)
tpuSensorsRewired       = true;   % true = Transform Sensors measure the TPU spring (guide step 6)
runGeometryCheck        = true;
runPartA                = true;
runPartB                = true;
useParallel             = false;  % true needs Parallel Computing Toolbox (forces saveModelAfterEdits)
confirmLongRuns         = true;   % ask before starting a sweep predicted to take > 2 h

% ---- simulation ----
T_end   = 150;     % s   total simulated time
maxStep = 0.005;   % s   solver max step: guarantees the solver cannot "step over" a 50 ms pebble

% ---- rover geometry constants (copied from the model; metres) ----
g.rockerX       = 0.345;    % rocker pivot / wheel lateral position (+/-)
g.rockerEndY    = 0.235;    % rocker end (wheel) position along the rover
g.rockerAttachZ = -0.105;   % underside of the rocker end (attach frame F2)
g.L_up          = 0.22;     % m   spring/damper #2 between rocker and wheel block (your 22 cm)
g.rimH          = 0.05;     % m   rim = wheel block height (5 cm)
g.L_tpu         = 0.15;     % m   lower 15 cm of the wheel = TPU spring #1
g.L0_strut      = 0.2528;   % m   assembled length of Spring 1 / Spring 2 (computed from the model)

% ---- suspension parameters ----
k_tpu = 50000;   c_tpu = 500;    % TPU spring #1 (ground <-> wheel block)      EDIT to your TPU
k_up  = k_tpu;   c_up  = c_tpu;  % spring #2 (wheel block <-> rocker). Placeholder = TPU values. EDIT to your real values
kRange = [6750 6850];            % N/m    main strut stiffness swept
cRange = [150  250];             % N*s/m  main strut damping swept
nFull   = 10;                    % Part A grid points per axis (10x10 = 100 runs).  See run-time estimate!
nSample = 3;                     % Part B: nSample x nSample combinations
k_fixed = mean(kRange);  c_fixed = mean(cRange);   % used for the 0 mm check

% ---- targets (judged on the two 190 mm obstacles) ----
targetAngleDeg   = 5;      % deg  max allowed chassis angle change
targetSettleTime = 1;      % s    max settling time after the obstacle has passed
targetAngle      = deg2rad(targetAngleDeg);

% ---- terrain ("road") settings ----
ter.T_end       = T_end;
ter.dt          = 0.001;           % s      sample time of the terrain signal
ter.v           = 0.5;             % m/s    ASSUMED rover speed  (converts metres of ground into seconds)
ter.T_flat      = 10;              % s      0 mm at the start: rover settles + baseline is measured
ter.pebH        = [0.001 0.010];   % m      pebble height, uniform random 1-10 mm
ter.pebLen      = [0.03  0.08];    % m      pebble length along the ground (-> 60-160 ms at 0.5 m/s)
ter.pebMeanGap  = 1.5;             % s      average EXTRA flat time between pebbles (random)
ter.pebMinFlat  = 0.3;             % s      guaranteed flat 0 mm between two pebbles
ter.hObs        = 0.190;           % m      big obstacle height
ter.tObs        = [50 110];        % s      start of each 190 mm obstacle  (exactly two)
ter.obsRiseLen  = 0.20;            % m      ground length over which the obstacle rises (and falls)
ter.obsTopLen   = 0.30;            % m      length of the flat top (make it huge to get a "curb")
ter.obsOnWheel  = [true true];     % [left right]  which wheel meets the obstacle
ter.preFlat     = 5;               % s      0 mm before each obstacle (no pebbles)
ter.postFlat    = 10;              % s      0 mm after each obstacle (no pebbles)
ter.seed        = 1;               % random seed -> identical road in every run

% ---- metric settings ----
cfg.T_end         = T_end;   cfg.maxStep = maxStep;
cfg.T_flat        = ter.T_flat;  cfg.preFlat = ter.preFlat;  cfg.postFlat = ter.postFlat;
cfg.recoverWin    = ter.postFlat - 1;      % s  window after an obstacle used for settling time
cfg.settleBandFrac= 0.05;                  % settled = within 5 % of the peak ...
cfg.settleBandMin = deg2rad(0.05);         % ... but never tighter than 0.05 deg
cfg.k_tpu = k_tpu; cfg.c_tpu = c_tpu; cfg.k_up = k_up; cfg.c_up = c_up;
cfg.L0_tpu1 = g.L_tpu; cfg.L0_tpu2 = g.L_tpu;
cfg.tpuSensors = tpuSensorsRewired;
cfg.chunk      = 8;  if useParallel, cfg.chunk = 32; end
cfg.keepTraces = false;

%% ======================= 2. LOAD MODEL + APPLY GEOMETRY ==================
load_system(mdl);
if useParallel, saveModelAfterEdits = true; end   % workers load the model from disk
if applyModelEdits
    fprintf('Applying geometry numbers to the model...\n');
    local_apply_model_edits(mdl, g, matchStrutToInitialPose);
    if saveModelAfterEdits, save_system(mdl); fprintf('Model saved.\n'); end
end
local_preflight(mdl);
local_check_geometry(mdl, g, matchStrutToInitialPose);

%% ======================= 3. BUILD THE ROAD ===============================
terr = local_build_terrain(ter);
terrain1 = timeseries(terr.hL, terr.t);   terrain1.Name = 'terrain1';   % LEFT  wheel ground height [m]
terrain2 = timeseries(terr.hR, terr.t);   terrain2.Name = 'terrain2';   % RIGHT wheel ground height [m]

fprintf('Road built: %d pebbles left, %d pebbles right, %d obstacles of %.0f mm at t = %s s.\n', ...
    size(terr.pebL,1), size(terr.pebR,1), numel(terr.obsStart), ter.hObs*1000, mat2str(ter.tObs));
fprintf('           obstacle lasts %.2f s (rise %.2f s, top %.2f s, fall %.2f s)\n', ...
    terr.obsEnd(1)-terr.obsStart(1), ter.obsRiseLen/ter.v, ter.obsTopLen/ter.v, ter.obsRiseLen/ter.v);

figure('Name','1 - Terrain input (ground height under each wheel)');
subplot(3,1,1); plot(terr.t, terr.hL*1000,'b'); grid on; local_shade(terr);
ylabel('Left ground (mm)'); title('Ground height under the wheels - pebbles 1-10 mm, two 190 mm obstacles');
subplot(3,1,2); plot(terr.t, terr.hR*1000,'r'); grid on; local_shade(terr);
ylabel('Right ground (mm)'); xlabel('Time (s)');
subplot(3,1,3); zt = [terr.obsStart(1)-2, terr.obsEnd(1)+4];
plot(terr.t, terr.hL*1000,'b', terr.t, terr.hR*1000,'r--'); grid on; xlim(zt);
xlabel('Time (s)'); ylabel('mm'); title('Zoom: first 190 mm obstacle'); legend('left','right');
drawnow;

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
        warning(['The rover does NOT stay put at 0 mm input (moved %.2f deg). Most likely causes: ' ...
            '(1) Spring 1/2 natural length is not the assembled length -> set matchStrutToInitialPose = true; ' ...
            '(2) a guide joint is not vertical -> re-check guide steps 1-4; (3) TPU gap wrong (see next line).'], moveDeg);
    end
    if lateDeg > 0.05
        warning('Still moving at the end of the 0 mm check -> raise ter.T_flat or check damping.');
    end
    if tpuSensorsRewired
        try
            for side = 1:2
                [~, gp] = local_get_signal(outChk, sprintf('tpu_gap%d', side));
                gp = gp(:,1) * 0.01;                                   % converters output cm -> m
                fprintf('  TPU spring %d length: start %.4f m, end %.4f m (natural %.3f m)\n', side, gp(1), gp(end), g.L_tpu);
                if abs(gp(1) - g.L_tpu) > 0.01
                    warning('TPU spring %d starts %.3f m long, expected %.3f m -> the sphere is not 15 cm under the wheel (guide steps 1 & 6).', side, gp(1), g.L_tpu);
                end
            end
        catch ME
            warning('Could not read tpu_gap1/2 (%s). Set tpuSensorsRewired = false or check the To Workspace blocks.', ME.message);
            cfg.tpuSensors = false;
        end
    end
    figure('Name','2 - 0 mm check (rover must stay at its initial pose)');
    plot(tq, rad2deg(aq - aq(1))); grid on; xlabel('Time (s)'); ylabel('Chassis angle change (deg)');
    title('0 mm input: chassis angle should move only a tiny amount and then flatten'); drawnow;
end

%% ======================= 5. RUN-TIME ESTIMATE ============================
nRuns = 0;  if runPartA, nRuns = nRuns + nFull^2; end;  if runPartB, nRuns = nRuns + nSample^2; end
if ~isnan(tCheck) && nRuns > 0
    perRun = tCheck * (T_end / Tchk);          % pessimistic: includes compile time of the short run
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

    peakGrid   = reshape([resA.peakObs],  size(KK));     % rad
    settleGrid = reshape([resA.settleObs], size(KK));    % s
    rmsGrid    = reshape([resA.rmsPeb],   size(KK));     % rad
    passGrid   = (peakGrid <= targetAngle) & (settleGrid <= targetSettleTime);
    nFail = sum(~cellfun(@isempty, {resA.err}));
    fprintf('Part A done. %d of %d combinations met BOTH targets (<=%g deg AND <=%g s). %d runs failed to execute.\n', ...
        sum(passGrid(:)), numel(KK), targetAngleDeg, targetSettleTime, nFail);
    if nFail > 0
        firstErr = resA(find(~cellfun(@isempty, {resA.err}), 1)).err;
        warning('First failure message: %s', firstErr);
    end

    figure('Name','3 - Part A: (k,c) sweep on the two 190 mm obstacles');
    tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    nexttile; imagesc(kVals, cVals, rad2deg(peakGrid)); set(gca,'YDir','normal'); colorbar;
    xlabel('k (N/m)'); ylabel('c (N\cdot s/m)'); title(sprintf('Peak chassis angle, worst obstacle (deg) - target %g', targetAngleDeg));
    nexttile; imagesc(kVals, cVals, settleGrid); set(gca,'YDir','normal'); colorbar;
    xlabel('k (N/m)'); ylabel('c (N\cdot s/m)'); title(sprintf('Settling time after obstacle (s) - target %g', targetSettleTime));
    nexttile; imagesc(kVals, cVals, rad2deg(rmsGrid)); set(gca,'YDir','normal'); colorbar;
    xlabel('k (N/m)'); ylabel('c (N\cdot s/m)'); title('RMS chassis angle on pebbles only (deg)');
    nexttile; imagesc(kVals, cVals, double(passGrid)); set(gca,'YDir','normal'); colorbar;
    xlabel('k (N/m)'); ylabel('c (N\cdot s/m)'); title('1 = meets both targets');
    title(tl, 'Full (k,c) sweep'); drawnow;
end

%% ======================= 7. PART B: side-by-side traces ==================
if runPartB
    kS = linspace(kRange(1), kRange(2), nSample);
    cS = linspace(cRange(1), cRange(2), nSample);
    [KS, CS] = meshgrid(kS, cS);   KS = KS'; CS = CS';        % row i = k, col j = c
    nCombos = numel(KS);
    fprintf('\nPart B: %d representative combinations...\n', nCombos);
    cfgB = cfg;  cfgB.keepTraces = true;
    resB = local_run_batch(mdl, cfgB, terr, KS(:)', CS(:)', useParallel, 'Part B');

    peakList = [resB.peakObs];  settleList = [resB.settleObs];
    passList = (peakList <= targetAngle) & (settleList <= targetSettleTime);

    fig4 = figure('Name','4 - Part B: chassis angle change, full 150 s');
    t1 = tiledlayout(fig4, nSample, nSample, 'TileSpacing','compact','Padding','compact');
    fig5 = figure('Name','5 - Part B: zoom on first 190 mm obstacle');
    t2 = tiledlayout(fig5, nSample, nSample, 'TileSpacing','compact','Padding','compact');
    for idx = 1:nCombos
        st = "FAIL";  if passList(idx), st = "PASS"; end
        ttl = sprintf('k=%.0f, c=%.0f [%s]\npeak=%.2f deg, settle=%.2f s', KS(idx), CS(idx), st, ...
            rad2deg(peakList(idx)), settleList(idx));
        for f = 1:2
            if f == 1, ax = nexttile(t1); else, ax = nexttile(t2); end
            r = resB(idx);
            if ~isempty(r.err) || isempty(r.trace_t)
                title(ax, sprintf('k=%.0f, c=%.0f\n[FAILED]', KS(idx), CS(idx)), 'FontSize', 8);  continue;
            end
            plot(ax, r.trace_t, rad2deg(r.trace_dev), 'LineWidth', 1); hold(ax, 'on'); grid(ax, 'on');
            yline(ax, targetAngleDeg, ':m');  yline(ax, -targetAngleDeg, ':m');
            for j = 1:numel(terr.obsStart)
                xline(ax, terr.obsStart(j), ':r');  xline(ax, terr.obsEnd(j), ':r');
            end
            if f == 2, xlim(ax, [terr.obsStart(1)-2, terr.obsEnd(1)+cfg.recoverWin]); end
            title(ax, ttl, 'FontSize', 8);  xlabel(ax, 'Time (s)');  ylabel(ax, '\Delta angle (deg)');
        end
    end
    title(t1, 'Chassis angle change from the 0 mm rest angle (red dotted = obstacle, magenta = target band)');
    title(t2, 'Zoom on obstacle 1');

    validIdx = isfinite(peakList) & isfinite(settleList);
    if any(validIdx)
        settleScore = min(settleList, cfg.recoverWin);          % Inf -> window length
        cand = find(passList);  header = 'Best PASSING combination';
        if isempty(cand), cand = find(validIdx);  header = 'No sampled combination met both targets - closest one'; end
        score = 0.5*peakList(cand)/max(peakList(validIdx)) + 0.5*settleScore(cand)/max(settleScore(validIdx));
        [~, ib] = min(score);  best = cand(ib);
        fprintf('\n%s (of %d sampled):\n  k = %.1f N/m, c = %.1f N*s/m\n', header, nCombos, KS(best), CS(best));
        fprintf('  worst-obstacle peak = %.2f deg, worst settling time = %.2f s, pebble RMS = %.3f deg\n', ...
            rad2deg(peakList(best)), settleList(best), rad2deg(resB(best).rmsPeb));
        for j = 1:numel(terr.obsStart)
            fprintf('    obstacle %d (t=%g s): peak %.2f deg, settle %.2f s\n', j, terr.obsStart(j), ...
                rad2deg(resB(best).peak(j)), resB(best).ts(j));
        end
        if cfg.tpuSensors && all(isfinite(resB(best).tpuMin))
            fprintf('  smallest TPU spring length during the run: left %.3f m, right %.3f m (natural %.2f m)\n', ...
                resB(best).tpuMin(1), resB(best).tpuMin(2), g.L_tpu);
            if min(resB(best).tpuMin) < 0.03
                warning('TPU spring almost fully compressed -> the wheel "bottoms out"; results near obstacles are not trustworthy.');
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
% Ground height [m] under each wheel: 0 mm flat + random pebbles + exactly numel(p.tObs) big obstacles.
t = (0:p.dt:p.T_end)';
n = numel(t);
Tr   = p.obsRiseLen / p.v;               % s  rise time
Tt   = p.obsTopLen  / p.v;               % s  time on top
Tobs = 2*Tr + Tt;                        % s  whole obstacle
obsStart = p.tObs(:)';
obsEnd   = obsStart + Tobs;
if any(obsStart - p.preFlat < p.T_flat), error('First obstacle starts too early: need tObs >= T_flat + preFlat.'); end
if any(obsEnd + p.postFlat > p.T_end),   error('Last obstacle ends too late: need tObs + duration + postFlat <= T_end.'); end
if numel(obsStart) > 1 && any(diff(obsStart) < Tobs + p.postFlat + p.preFlat), error('Obstacles overlap their flat guard zones.'); end
excl = [obsStart' - p.preFlat, obsEnd' + p.postFlat];       % protected windows: no pebbles here

% smooth (raised-cosine) obstacle: rise - flat top - fall.  Smooth = the solver gets clean velocity/acceleration.
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
% Random pebbles: each one is a smooth half-wave bump [start width height].
h   = zeros(size(t));
peb = zeros(0,3);
tc  = p.T_flat;
while true
    tc = tc + p.pebMinFlat - p.pebMeanGap * log(1 - rand());  % flat gap before this pebble
    w  = (p.pebLen(1) + (p.pebLen(2)-p.pebLen(1))*rand()) / p.v;
    H  = p.pebH(1)   + (p.pebH(2)-p.pebH(1))*rand();
    if tc + w > p.T_end - 1, break; end
    hit = find(tc < excl(:,2) & (tc + w) > excl(:,1), 1);
    if ~isempty(hit)                                          % would touch an obstacle zone -> jump past it
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
% All numbers are relative to the chassis angle at 0 mm input (last 2 s of the initial flat period).
m = local_empty_metrics(numel(terr.obsStart));
[t, a] = local_get_signal(out, 'link3_angle');  a = a(:,1);
[tu, iu] = unique(t);
tq = (0:0.001:cfg.T_end)';
aq = interp1(tu, a(iu), tq, 'linear', 'extrap');

iB = tq >= cfg.T_flat-2 & tq < cfg.T_flat;
m.a0    = mean(aq(iB));
m.drift = abs(mean(aq(tq >= cfg.T_flat-2 & tq < cfg.T_flat-1)) - mean(aq(tq >= cfg.T_flat-1 & tq < cfg.T_flat)));
dev = aq - m.a0;

for j = 1:numel(terr.obsStart)
    t0 = terr.obsStart(j);  t1 = terr.obsEnd(j);
    base = mean(aq(tq >= t0-2 & tq < t0-0.2));                    % rest angle just before this obstacle
    win  = tq >= t0 & tq <= t1 + cfg.recoverWin;
    m.peak(j) = max(abs(aq(win) - base));
    iS   = tq >= t1 & tq <= t1 + cfg.recoverWin;
    fin  = mean(aq(tq > t1 + cfg.recoverWin - 1 & tq <= t1 + cfg.recoverWin));   % where it finally rests
    band = max(cfg.settleBandFrac * m.peak(j), cfg.settleBandMin);
    outside = abs(aq(iS) - fin) > band;
    tS = tq(iS);
    if ~any(outside),      m.ts(j) = 0;
    elseif outside(end),   m.ts(j) = Inf;                          % never settled inside the window
    else,                  m.ts(j) = tS(find(outside, 1, 'last')) - t1;
    end
end
m.peakObs   = max(m.peak);
m.settleObs = max(m.ts);

mask = tq >= cfg.T_flat;                                            % pebble-only region
for j = 1:numel(terr.obsStart)
    mask(tq >= terr.obsStart(j) - cfg.preFlat & tq <= terr.obsEnd(j) + cfg.postFlat) = false;
end
m.rmsPeb  = sqrt(mean(dev(mask).^2));
m.peakPeb = max(abs(dev(mask)));

if cfg.keepTraces
    m.trace_t   = tq(1:10:end);                                     % 100 Hz is plenty for plots
    m.trace_dev = dev(1:10:end);
end
if cfg.tpuSensors
    try
        for side = 1:2
            [~, gp] = local_get_signal(out, sprintf('tpu_gap%d', side));
            m.tpuMin(side) = min(gp(:,1)) * 0.01;                   % cm -> m
        end
    catch
        m.tpuMin = [NaN NaN];
    end
end
end

function m = local_empty_metrics(nObs)
m = struct('peakObs', NaN, 'settleObs', NaN, 'peak', NaN(1,nObs), 'ts', NaN(1,nObs), ...
    'rmsPeb', NaN, 'peakPeb', NaN, 'drift', NaN, 'a0', NaN, 'tpuMin', [NaN NaN], ...
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
% Reads a logged signal whether the To Workspace block saves Timeseries, Array or Structure.
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
    'SimscapeLogType', 'none');           % no Simscape logging: far less memory over many runs
end

function res = local_run_batch(mdl, cfg, terr, kList, cList, useParallel, label)
% Runs the (k,c) pairs in small chunks and keeps only the metrics (not the raw simulation data).
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
% Writes the geometry numbers of the step-by-step guide into the model (in memory).
% All values are metres. "SID" = the block's permanent id inside the .slx file.
zBase = g.rockerAttachZ - g.L_up - g.rimH;     % height of the ground-joint base = underside of wheel block

% --- guide frame under each rocker end: undoes the pivot's 90 deg tilt so Z points UP, L_up below the rocker end
for sid = [91 131]
    local_set_rt(mdl, sid, 'StandardAxis', '+Y', 90, 'Cartesian', [0 0 g.L_up]);
end
% --- lower end of spring #2, L_up below the wheel-top frame
for sid = [141 144]
    local_set_rt(mdl, sid, 'None', '', 0, 'Cartesian', [0 0 -g.L_up]);
end
% --- ground joint bases: vertical axis (no rotation), directly under each wheel
local_set_rt(mdl, 152, 'None', '', 0, 'Cartesian', [-g.rockerX g.rockerEndY zBase]);   % left
local_set_rt(mdl, 156, 'None', '', 0, 'Cartesian', [ g.rockerX g.rockerEndY zBase]);   % right
% --- sphere sits 15 cm below the joint follower frame (TPU spring natural length)
for sid = [151 155]
    local_set_rt(mdl, sid, 'None', '', 0, 'Cartesian', [0 0 -g.L_tpu]);
end
% --- spring #2 (rocker <-> wheel block): natural length L_up, own stiffness/damping variables (k_up, c_up)
for sid = [138 142]
    h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, sid));
    set_param(h, 'NaturalLength', num2str(g.L_up), 'NaturalLengthUnits', 'm', ...
        'SpringStiffness', 'k_up', 'DampingCoefficient', 'c_up');
end
% --- wheel blocks: 5 cm tall (rim)
for sid = [137 145]
    h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, sid));
    set_param(h, 'BrickDimensions', mat2str([10 10 g.rimH*100]), 'BrickDimensionsUnits', 'cm');
end
% --- main struts: natural length = assembled length -> zero force in the initial pose
if matchStrut
    for sid = [3 24]
        h = Simulink.ID.getHandle(sprintf('%s:%d', mdl, sid));
        set_param(h, 'NaturalLength', num2str(g.L0_strut, '%.5f'), 'NaturalLengthUnits', 'm');
    end
end
% --- From Workspace blocks: left ground = terrain1 (SID 177), right ground = terrain2 (SID 176)
set_param(Simulink.ID.getHandle(sprintf('%s:%d', mdl, 177)), 'VariableName', 'terrain1');
set_param(Simulink.ID.getHandle(sprintf('%s:%d', mdl, 176)), 'VariableName', 'terrain2');
% --- Simulink-PS converters that feed the ground joints: 2nd order filter (position + velocity + acceleration)
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
% Friendly early checks so a wiring mistake shows up as a clear message, not a cryptic solver error.
fw = cellstr(find_system(mdl, 'SearchDepth', 1, 'BlockType', 'FromWorkspace'));
names = {};
for i = 1:numel(fw)
    if ~isempty(fw{i}), names{end+1} = strtrim(get_param(fw{i}, 'VariableName')); end %#ok<AGROW>
end
if ~all(ismember({'terrain1', 'terrain2'}, names))
    error(['The model needs two From Workspace blocks with Variable name  terrain1  and  terrain2 ' ...
        '(guide step 5). Found: %s'], strjoin(names, ', '));
end
old = [find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Step'); ...
       find_system(mdl, 'SearchDepth', 1, 'BlockType', 'UniformRandomNumber')];
if ~isempty(old)
    warning('The old Step / Uniform Random Number blocks are still in the model (%d found). Delete them (guide step 5) - they need variables that no longer exist.', numel(old));
end
end

function local_shade(terr)
% shades the two obstacle periods on the current axes
yl = ylim;
for j = 1:numel(terr.obsStart)
    patch([terr.obsStart(j) terr.obsEnd(j) terr.obsEnd(j) terr.obsStart(j)], [yl(1) yl(1) yl(2) yl(2)], ...
        [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end
set(gca, 'Children', flipud(get(gca, 'Children')));
end

function local_check_geometry(mdl, g, matchStrut)
% Warns if the numbers typed at the top of this script differ from what is actually inside the model.
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
            'Set applyModelEdits = true (script writes it into the model) or edit the block by hand.'], chk{i,3}, v, chk{i,2});
    end
end
end