function ody_review(run_name)
% ODY_REVIEW  Parse, load, and plot one Odyssey simulator run.
%
%   ody_review             % auto-selects the newest .obd in SData\
%   ody_review('S2609501') % specific run
%
% Parses the .obd file if needed, loads all data and sensor column-index
% variables directly into the BASE workspace, then calls sim_review.
%
% After returning, the workspace contains 'data', 'run_name', 'm_depth',
% 'time', etc. — ready for further interactive use.

if nargin < 1
    mfile = ody_parse();
else
    mfile = ody_parse(run_name);
end

% Derive .mat path from the .m path ody_parse just wrote
[d, n] = fileparts(mfile);
mat_file = fullfile(d, [n, '.mat']);

if ~exist(mat_file, 'file')
    error('ody_review: mat file not found: %s', mat_file);
end

% ---- load into function workspace first, then push to base --------------
% This avoids any cd / path dependency — we use the full path to the .mat.
s = load(mat_file);   % gives s.data, s.col_vars, s.run_str, s.mission_start

% Push data matrix
assignin('base', 'data',          s.data);
assignin('base', 'run_name',      s.run_str);
assignin('base', 'mission_start', s.mission_start);

% Push every sensor column-index variable  (m_depth, m_pitch, etc.)
fns = fieldnames(s.col_vars);
for i = 1:numel(fns)
    assignin('base', fns{i}, s.col_vars.(fns{i}));
end

fprintf('ody_review: loaded %s — %d cycles, %d sensors\n', ...
        s.run_str, size(s.data,1), numel(fns));

% ---- call sim_review in the base workspace ------------------------------
evalin('base', 'sim_review');

end
