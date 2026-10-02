%% stair_descent_single_run.m
% =========================================================================
% ONE simulation: rover driving DOWN a long staircase.
%   - step height 200 mm, rover speed 0.5 m/s, 100 s
%   - LEFT and RIGHT wheel get exactly the same ground input
%   - main strut fixed at k = 2000 N/m, c = 325 N*s/m
%   - TPU / spring #2 properties unchanged from the previous scripts
%   - only the CHASSIS ANGLE is plotted
%
% USE WITH: simulation_actualrover_FIXED.slx (From Workspace blocks read
%   terrain1 / terrain2, To Workspace block logs link3_angle, tpu_gap1/2)
%
% IMPORTANT - GRAVITY MUST BE ON in the model (Mechanism Configuration,
%   Uniform Gravity [0 0 -9.81]). The ground is a moved sphere joined to the
%   wheel by a spring/damper. Only gravity makes the wheel fall away from the
%   edge; with gravity off the TPU spring would drag the wheel down.
% =========================================================================
clear; clc; close all;

%% ======================= 1. USER SETTINGS ================================
mdl = 'simulation_actualrover_FIXED';

% ---- suspension (main strut is the one you are testing) ----
k = 2000;                 % N/m      main strut stiffness
c = 325;                  % N*s/m    main strut damping
k_tpu = 50000;  c_tpu = 500;          % TPU spring #1 (same tire properties as before)
k_up  = k_tpu;  c_up  = c_tpu;        % spring #2 - still the placeholders, EDIT to your real values
L0_tpu = 0.15;                        % m  TPU spring natural length

% ---- simulation ----
T_end   = 100;            % s
maxStep = 0.005;          % s   solver max step (edge drop lasts ~0.3 s, so this is plenty)
T_flat  = 10;             % s   flat ground at the start: rover settles under gravity, baseline angle measured

% ---- staircase ----
stair.h        = 0.200;   % m     step height (drop per step)
stair.v        = 0.5;     % m/s   rover speed
stair.tread    = 0.30;    % m     tread (horizontal step) length - ASSUMED, not given. EDIT to your stair.
stair.dropTime = pi*sqrt(stair.h/(2*9.81));  % s  time over which one edge drops (~0.32 s).
                          %        Chosen so the ground never accelerates downward faster than gravity;
                          %        a faster drop would make the TPU spring pull the wheel down.
stair.t0       = T_flat;  % s     time of the first edge
stair.nSteps   = Inf;     % number of steps (Inf = stairs continue to the end; 1 = a single drop)
stair.dt       = 0.001;   % s     sample time of the ground signal
stair.frame    = 'rover'; % 'rover' : ground height measured in a frame that descends with the staircase
                          %           at its mean slope (constant velocity, so gravity is unaffected).
                          %           Stays bounded - USE THIS if the chassis/base of your model is
                          %           fixed to the world frame (it cannot travel 30 m downward).
                          % 'world' : raw staircase, ground goes down 200 mm every step, without limit.
                          %           Use only if the rover body is free to fall with the stairs,
                          %           or for a few steps (stair.nSteps small).

plotEdgesFromTo = [T_flat T_flat + 6];   % s   window of the zoom plot

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
Tstep = stair.tread / stair.v;                          % s  time per step
if stair.dropTime >= Tstep
    error('Edge drop time (%.2f s) must be shorter than the step time (%.2f s). Increase stair.tread or reduce stair.dropTime.', stair.dropTime, Tstep);
end
t = (0:stair.dt:T_end)';
s = t - stair.t0;
h = zeros(size(t));
on = s >= 0;
j = floor(s(on) / Tstep);                               % completed steps
r = s(on) - j*Tstep;                                    % time inside the current step
frac = ones(size(r));
ie = r < stair.dropTime;
frac(ie) = 0.5*(1 - cos(pi*r(ie)/stair.dropTime));      % smooth (raised-cosine) edge
n = min(j + frac, stair.nSteps);                        % steps descended so far
world = -stair.h * n;
if strcmp(stair.frame, 'rover')
    sEff = min(s(on), stair.nSteps*Tstep);
    h(on) = world + (stair.h/Tstep) * sEff;             % remove the mean descent -> bounded sawtooth
else
    h(on) = world;
end
nEdges = min(floor((T_end - stair.t0)/Tstep) + 1, stair.nSteps);
edgeT  = stair.t0 + (0:nEdges-1)*Tstep;                 % start time of every edge

% same input on both wheels
terrain1 = timeseries(h, t);  terrain1.Name = 'terrain1';
terrain2 = timeseries(h, t);  terrain2.Name = 'terrain2';

fprintf('Staircase: %.0f mm steps, tread %.2f m, %.2f s per step, %d edges in %g s.\n', ...
    stair.h*1000, stair.tread, Tstep, nEdges, T_end);
fprintf('Ground input range (frame = %s): %.0f mm to %.0f mm.\n', stair.frame, min(h)*1000, max(h)*1000);

%% ======================= 4. RUN (single simulation) ======================
in = Simulink.SimulationInput(mdl);
in = in.setVariable('k', k);              in = in.setVariable('c', c);
in = in.setVariable('k_tpu', k_tpu);      in = in.setVariable('c_tpu', c_tpu);
in = in.setVariable('k_up',  k_up);       in = in.setVariable('c_up',  c_up);
in = in.setVariable('L0_tpu1', L0_tpu);   in = in.setVariable('L0_tpu2', L0_tpu);
in = in.setVariable('terrain1', terrain1);
in = in.setVariable('terrain2', terrain2);
in = in.setModelParameter('StopTime', num2str(T_end), 'MaxStep', num2str(maxStep), 'SimscapeLogType', 'none');

fprintf('\nRunning k = %g N/m, c = %g N*s/m for %g s ...\n', k, c, T_end);
tic;
out = sim(in);
fprintf('Done in %.1f min.\n', toc/60);
if ~isempty(out.ErrorMessage), error('Simulation failed:\n%s', out.ErrorMessage); end

%% ======================= 5. CHASSIS ANGLE ================================
[ta, aa] = local_get_signal(out, 'link3_angle');  aa = aa(:,1);
[tu, iu] = unique(ta);
tq = (0:0.001:T_end)';
aq = interp1(tu, aa(iu), tq, 'linear', 'extrap');

a0  = mean(aq(tq >= T_flat-2 & tq < T_flat));           % rest angle before the first edge
dev = rad2deg(aq - a0);                                 % chassis angle change [deg]

rest = abs(mean(aq(tq >= T_flat-2 & tq < T_flat-1)) - mean(aq(tq >= T_flat-1 & tq < T_flat)));
if rad2deg(rest) > 0.05
    warning('Rover was still moving (%.3f deg) before the first edge -> increase T_flat (gravity sag still settling).', rad2deg(rest));
end

% per-step peak (each step window = from its edge to the next edge)
peakStep = NaN(1, nEdges);
for q = 1:nEdges
    w = tq >= edgeT(q) & tq < min(edgeT(q) + Tstep, T_end);
    if any(w), peakStep(q) = max(abs(dev(w))); end
end

fprintf('\n---- Chassis angle change (relative to rest angle at t = %g s) ----\n', T_flat);
fprintf('  max  : %+7.2f deg\n  min  : %+7.2f deg\n  peak-to-peak: %.2f deg\n  RMS  : %.2f deg\n', ...
    max(dev), min(dev), max(dev)-min(dev), sqrt(mean(dev(tq >= T_flat).^2)));
fprintf('  per-step peak |angle|: first step %.2f deg, median %.2f deg, worst %.2f deg\n', ...
    peakStep(1), median(peakStep,'omitnan'), max(peakStep));

try
    for side = 1:2
        [~, gp] = local_get_signal(out, sprintf('tpu_gap%d', side));
        gp = gp(:,1) * 0.01;                            % cm -> m
        fprintf('  TPU spring %d length: min %.3f m, max %.3f m (natural %.2f m)\n', side, min(gp), max(gp), L0_tpu);
        if min(gp) < 0.03
            warning('TPU spring %d almost fully compressed -> wheel bottoms out.', side);
        end
    end
catch
    fprintf('  (tpu_gap signals not found - TPU length not reported)\n');
end

%% ======================= 6. PLOT (chassis angle only) ====================
figure('Name', 'Chassis angle - stair descent', 'Position', [100 100 1100 650]);
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(tq, dev, 'LineWidth', 1); grid on; hold on;
xline(T_flat, ':r');
xlabel('Time (s)'); ylabel('\Delta chassis angle (deg)');
title(sprintf('Full run: k = %g N/m, c = %g N\\cdot s/m, %.0f mm steps at %.1f m/s (red = first edge)', k, c, stair.h*1000, stair.v));

nexttile;
zi = tq >= plotEdgesFromTo(1) & tq <= plotEdgesFromTo(2);
plot(tq(zi), dev(zi), 'LineWidth', 1.2); grid on; hold on;
for q = 1:nEdges
    if edgeT(q) >= plotEdgesFromTo(1) && edgeT(q) <= plotEdgesFromTo(2)
        xline(edgeT(q), ':r');
    end
end
xlim(plotEdgesFromTo);
xlabel('Time (s)'); ylabel('\Delta chassis angle (deg)');
title('Zoom on the first steps (red dotted = start of each edge)');
title(tl, 'Chassis angle during stair descent');
drawnow;

%% ======================= LOCAL FUNCTION ==================================
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
