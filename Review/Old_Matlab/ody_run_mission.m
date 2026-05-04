function result = ody_run_mission(mission_file, varargin)
% ODY_RUN_MISSION  Run the Odyssey simulator for one mission and parse output.
%
%   result = ody_run_mission(mission_file)
%   result = ody_run_mission(mission_file, 'param', value, ...)
%
% INPUTS
%   mission_file  : name (no extension needed) or full path to the mission
%                   file.  If no directory is given, cfg.mission_dir is used.
%
% OPTIONAL NAME-VALUE PAIRS
%   'overrides'   : struct of mission-file parameter overrides applied
%                   before running (see ody_write_mission.m).  Default: [].
%   'run_label'   : short string appended to the data filename for
%                   identification.  Default: ''.
%   'parse_optset': override cfg.parse_optset for this run.
%   'verbose'     : 0|1, print progress.  Default: 1.
%   'timeout'     : override cfg.sim_timeout_s for this run.
%
% OUTPUT  result struct with fields:
%   .mission_file   full path of mission file that was run
%   .data_file      full path of binary data file produced
%   .dat_file       full path of ASCII .dat file (after parse)
%   .m_file         full path of Matlab index .m file (after parse)
%   .run_label      label used
%   .exit_code      simulator exit code (0 = success)
%   .parse_ok       true if parse succeeded
%   .elapsed_s      wall-clock seconds for simulation
%   .data           loaded data struct (from ody_load_data), or []

cfg = ody_config();

% ---- parse optional arguments -------------------------------------------
p = inputParser;
addRequired(p,  'mission_file', @ischar);
addParameter(p, 'overrides',    [],   @(x) isstruct(x)||isempty(x));
addParameter(p, 'run_label',    '',   @ischar);
addParameter(p, 'parse_optset', cfg.parse_optset, @ischar);
addParameter(p, 'verbose',      1,    @isnumeric);
addParameter(p, 'timeout',      cfg.sim_timeout_s, @isnumeric);
parse(p, mission_file, varargin{:});
opt = p.Results;

verbose = opt.verbose;

% ---- resolve paths -------------------------------------------------------
[mdir, mname, mext] = fileparts(opt.mission_file);
if isempty(mdir)
    mdir = cfg.mission_dir;
end
if isempty(mext)
    mext = '.mis';   % default extension; adjust if yours differs
end
mission_full = fullfile(mdir, [mname, mext]);

if ~exist(mission_full, 'file')
    error('ody_run_mission: mission file not found: %s', mission_full);
end

% ---- apply overrides to a temporary copy ---------------------------------
if ~isempty(opt.overrides)
    tmp_name = sprintf('%s_tmp_%s', mname, opt.run_label);
    if isempty(opt.run_label)
        tmp_name = sprintf('%s_tmp_%d', mname, round(now*1e6));
    end
    tmp_mission = fullfile(mdir, [tmp_name, mext]);
    ody_write_mission(mission_full, tmp_mission, opt.overrides);
    run_mission = tmp_mission;
    run_name    = tmp_name;
else
    run_mission = mission_full;
    run_name    = mname;
end

if verbose
    fprintf('[ody_run_mission] Running: %s\n', run_mission);
end

% ---- invoke simulator ----------------------------------------------------
% The exe must be run from the data directory so it writes output there.
old_dir = cd(cfg.data_dir);
cmd = sprintf('"%s" "%s"', cfg.sim_exe, run_mission);

t0 = tic;
[exit_code, sim_output] = system(cmd);
elapsed = toc(t0);
cd(old_dir);

if verbose
    fprintf('[ody_run_mission] Finished in %.1f s, exit code %d\n', ...
            elapsed, exit_code);
end
if exit_code ~= 0
    warning('ody_run_mission: simulator returned exit code %d\n%s', ...
            exit_code, sim_output);
end

% ---- find the data file --------------------------------------------------
% The simulator names the data file from the mission name; adjust the glob
% pattern below if your build uses a different convention.
pat = fullfile(cfg.data_dir, [run_name, '*.dat']);
hits = dir(pat);
if isempty(hits)
    % Fallback: find newest file in data dir
    all  = dir(fullfile(cfg.data_dir, '*.dat'));
    [~, idx] = max([all.datenum]);
    hits = all(idx);
end

if isempty(hits)
    error('ody_run_mission: cannot find data file for run "%s" in %s', ...
          run_name, cfg.data_dir);
end
% pick newest match
[~, idx] = max([hits.datenum]);
data_file = fullfile(cfg.data_dir, hits(idx).name);

if verbose
    fprintf('[ody_run_mission] Data file: %s\n', data_file);
end

% ---- run parse -----------------------------------------------------------
parse_ok = false;
[fdir, fname_base] = fileparts(data_file);
optset_arg = '';
if ~isempty(opt.parse_optset)
    optset_arg = sprintf('-o %s', opt.parse_optset);
end
parse_cmd = sprintf('"%s" %s "%s"', cfg.parse_exe, optset_arg, data_file);

old_dir2 = cd(fdir);
[pc, ~] = system(parse_cmd);
cd(old_dir2);

dat_file = fullfile(fdir, [fname_base, '.dat']);
m_file   = fullfile(fdir, [fname_base, '.m']);

if pc == 0 && exist(dat_file,'file') && exist(m_file,'file')
    parse_ok = true;
    if verbose
        fprintf('[ody_run_mission] Parse OK → %s\n', dat_file);
    end
else
    warning('ody_run_mission: parse failed (exit %d) or output missing', pc);
end

% ---- clean up temp mission file ------------------------------------------
if ~isempty(opt.overrides) && exist(run_mission, 'file')
    delete(run_mission);
end

% ---- load data -----------------------------------------------------------
data = [];
if parse_ok
    try
        data = ody_load_data(dat_file, m_file);
    catch ME
        warning('ody_run_mission: ody_load_data failed: %s', ME.message);
    end
end

% ---- assemble result -----------------------------------------------------
result.mission_file = mission_full;
result.data_file    = data_file;
result.dat_file     = dat_file;
result.m_file       = m_file;
result.run_label    = opt.run_label;
result.exit_code    = exit_code;
result.parse_ok     = parse_ok;
result.elapsed_s    = elapsed;
result.data         = data;

end
