function export_model_architecture(mdl, outFile)
% EXPORT_MODEL_ARCHITECTURE  Write a readable description of a Simulink /
% Simscape Multibody model to a text file.
%
%   export_model_architecture                       % uses the defaults below
%   export_model_architecture('myModel', 'out.txt')
%
% Sections written:
%   1. Model settings (solver, gravity-related callbacks, model workspace)
%   2. Every block: full name, type, library source (e.g. sm_lib/Joints/Revolute Joint)
%   3. Connections: for every block, which blocks are attached to each port
%   4. Parameters of every block (dialog parameters and values)
%
% Upload the resulting .txt file so the model can be rebuilt or patched by script.
% Note: for Multibody blocks, LConn1/RConn1 are the left/right ports as drawn.
% Which one is Base (B) and which is Follower (F) depends on block orientation;
% the Orientation and Position lines are included so this can be worked out.

if nargin < 1, mdl = 'simulation_actualrover_FIXED'; end
if nargin < 2, outFile = [mdl '_architecture.txt']; end

load_system(mdl);
blks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'Block');

fid = fopen(outFile, 'w');
if fid < 0, error('Cannot open %s for writing.', outFile); end
cleaner = onCleanup(@() fclose(fid));

%% 1. model settings
fprintf(fid, '=== MODEL: %s (MATLAB %s) ===\n\n', mdl, version);
fprintf(fid, '--- Settings ---\n');
for p = {'Solver','SolverType','StopTime','MaxStep','RelTol','AbsTol','InitFcn','PreLoadFcn','PostLoadFcn'}
    try
        v = get_param(mdl, p{1});
        fprintf(fid, '%s = %s\n', p{1}, local_str(v));
    catch
    end
end
try
    mws = get_param(mdl, 'ModelWorkspace');
    d = mws.data;
    fprintf(fid, '\n--- Model workspace variables ---\n');
    for i = 1:numel(d)
        fprintf(fid, '%s = %s\n', d(i).Name, local_str(d(i).Value));
    end
catch
end

%% 2. block list
fprintf(fid, '\n\n=== BLOCKS (%d) ===\n', numel(blks));
for i = 1:numel(blks)
    b = blks{i};
    fprintf(fid, '\n[%d] %s\n', i, b);
    for p = {'BlockType','MaskType','ReferenceBlock','Orientation','Position'}
        try
            fprintf(fid, '    %s: %s\n', p{1}, local_str(get_param(b, p{1})));
        catch
        end
    end
end

%% 3. connections
fprintf(fid, '\n\n=== CONNECTIONS (block -> connected blocks, per port) ===\n');
for i = 1:numel(blks)
    b = blks{i};
    try
        pc = get_param(b, 'PortConnectivity');
    catch
        continue;
    end
    if isempty(pc), continue; end
    fprintf(fid, '\n%s\n', b);
    for j = 1:numel(pc)
        tgt = [pc(j).SrcBlock(:); pc(j).DstBlock(:)];
        names = {};
        for h = tgt'
            if h > 0
                try, names{end+1} = getfullname(h); end %#ok<TRYNC,AGROW>
            end
        end
        if isempty(names), names = {'(unconnected)'}; end
        fprintf(fid, '    %-10s -> %s\n', pc(j).Type, strjoin(names, ' | '));
    end
end

%% 4. parameters
fprintf(fid, '\n\n=== PARAMETERS ===\n');
for i = 1:numel(blks)
    b = blks{i};
    try
        dp = get_param(b, 'DialogParameters');
    catch
        continue;
    end
    if isempty(dp), continue; end
    fprintf(fid, '\n%s\n', b);
    fn = fieldnames(dp);
    for j = 1:numel(fn)
        try
            v = get_param(b, fn{j});
            fprintf(fid, '    %s = %s\n', fn{j}, local_str(v));
        catch
        end
    end
end

fprintf('Wrote %s. Upload it so the model can be patched or rebuilt by script.\n', outFile);
end

function s = local_str(v)
if ischar(v) || isstring(v)
    s = char(v);
elseif isnumeric(v) || islogical(v)
    s = mat2str(v);
else
    try
        s = evalc('disp(v)');
        s = strtrim(s);
    catch
        s = '<unprintable>';
    end
end
s = strrep(s, newline, ' ');
end
