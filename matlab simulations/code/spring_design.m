function T = spring_design(k_target, F_max, stroke, damperOD, varargin)
% SPRING_DESIGN  Helical compression spring candidates for a target stiffness.
%
%   T = spring_design(k_target, F_max, stroke, damperOD)
%
%   k_target  N/m   axial spring rate needed at the strut (from the (k,c) sweep)
%   F_max     N     largest spring force you expect (preload + k*stroke + shock margin)
%   stroke    m     working deflection of the strut
%   damperOD  m     outer diameter of the damper body the spring sits around
%                   (use [] if the spring sits beside the damper instead)
%
%   Optional name-value pairs:
%     'G'       shear modulus, Pa            (default 79.3e9, check your wire datasheet)
%     'A','m'   tensile strength fit sigma_ut = A/d^m, MPa and d in mm
%               (defaults 2211 and 0.145 = music wire ASTM A228; check Shigley/your supplier)
%     'tauFrac' allowable shear / sigma_ut   (default 0.45, static; no fatigue check)
%     'clash'   clash allowance as fraction of stroke (default 0.15)
%     'wires'   candidate wire diameters, m  (default 2 to 10 mm in 0.5 mm steps)
%     'Lmax'    maximum free length, m       (default 0.25)
%
%   Ends are assumed closed and ground: Nt = Na + 2.
%   Returns a table of candidates that satisfy all checks, best safety factor first.

p = inputParser;
addParameter(p, 'G', 79.3e9);
addParameter(p, 'A', 2211);
addParameter(p, 'm', 0.145);
addParameter(p, 'tauFrac', 0.45);
addParameter(p, 'clash', 0.15);
addParameter(p, 'wires', (2:0.5:10)*1e-3);
addParameter(p, 'Lmax', 0.25);
parse(p, varargin{:});
o = p.Results;

rows = [];
for d = o.wires
    sigUT  = o.A / (d*1000)^o.m * 1e6;          % Pa
    tauAll = o.tauFrac * sigUT;                 % Pa
    for C = 4:0.25:12                           % spring index D/d
        D  = C*d;
        Na = o.G * d^4 / (8 * D^3 * k_target);  % active coils from k = G d^4 / (8 D^3 Na)
        if Na < 3, continue; end                % too few coils for a stable rate
        Na = round(Na*4)/4;                     % quarter-coil resolution
        kAct = o.G * d^4 / (8 * D^3 * Na);
        Kw   = (4*C - 1)/(4*C - 4) + 0.615/C;   % Wahl factor
        tau  = Kw * 8 * F_max * D / (pi * d^3);
        Hs   = (Na + 2) * d;                    % solid height
        L0   = Hs + stroke*(1 + o.clash);       % free length
        ID   = D - d;                           % inner diameter
        okID = isempty(damperOD) || ID >= damperOD + 0.003;   % 3 mm radial clearance on diameter
        okL  = L0 <= o.Lmax;
        okB  = (L0/D) <= 4 || ~isempty(damperOD);             % buckling: guided by damper if coaxial
        okSF = tau <= tauAll;
        if okID && okL && okB && okSF
            rows(end+1,:) = [d D Na kAct C tau/1e6 tauAll/1e6 tauAll/tau Hs L0 ID]; %#ok<AGROW>
        end
    end
end

if isempty(rows)
    warning('No candidate met all checks. Relax Lmax or stroke, or check k_target * stroke against F_max.');
    T = table();  return;
end
T = array2table(rows, 'VariableNames', {'d_m','D_m','Na','k_actual','C','tau_MPa','tauAllow_MPa','SF','Hs_m','L0_m','ID_m'});
T = sortrows(T, 'SF', 'descend');
disp(T(1:min(10, height(T)), :));
end
