function D = ody_load_data(dat_file, m_file)
% ODY_LOAD_DATA  Load Odyssey parsed data into a convenient struct.
%
%   D = ody_load_data(dat_file, m_file)
%   D = ody_load_data(dat_file)          % auto-finds matching .m file
%
% The parse utility produces two files from the simulator binary output:
%   <name>.dat   tab-delimited ASCII matrix (rows = time steps)
%   <name>.m     Matlab script that defines column indices and loads .dat
%
% ODY_LOAD_DATA runs the .m script in a private workspace to capture all
% column-index variables, then wraps the data matrix into a struct so that
% you can write  D.m_depth  instead of  data(:, M_DEPTH).
%
% OUTPUT struct D fields:
%   .raw          raw Nt×Nvar data matrix
%   .vars         cell array of variable names (length Nvar)
%   .t            time vector (seconds), i.e. D.raw(:, col_m_present_time)
%   .Nt           number of time steps
%   .Nvar         number of variables
%   .run_name     run label from the .m file
%   .dat_file     path of the .dat file
%   .m_file       path of the .m file
%   .<var_name>   one field per variable, e.g. D.m_depth, D.real_x, ...
%
% Additionally, convenience groupings are provided:
%   .pos          struct with .north, .east, .depth
%   .att          struct with .roll, .pitch, .yaw (radians)
%   .ctrl         struct with commanded and measured actuators

% ---- locate files --------------------------------------------------------
if nargin < 2 || isempty(m_file)
    [d, n] = fileparts(dat_file);
    m_file = fullfile(d, [n, '.m']);
end
if ~exist(dat_file, 'file')
    error('ody_load_data: .dat file not found: %s', dat_file);
end
if ~exist(m_file, 'file')
    error('ody_load_data: .m index file not found: %s', m_file);
end

% ---- run the .m file in a private workspace ------------------------------
% The .m file does two things:
%   1. Assigns column indices  (e.g.  M_DEPTH = 5;)
%   2. Loads the .dat file     (load('foo.dat'); data = foo; ...)
%
% We capture workspace variables by running it via evalin/feval trick.
ws = run_m_file_in_workspace(m_file, dat_file);

% ---- extract the data matrix ---------------------------------------------
% The .m file assigns  data = <run_name>;  after the load()
if isfield(ws, 'data')
    raw = ws.data;
elseif isfield(ws, 'run_name')
    vname = ws.run_name;
    if isfield(ws, vname)
        raw = ws.(vname);
    else
        error('ody_load_data: cannot find data matrix in workspace');
    end
else
    % last resort: find the largest numeric matrix
    fnames = fieldnames(ws);
    best = []; best_n = 0;
    for i = 1:numel(fnames)
        v = ws.(fnames{i});
        if isnumeric(v) && ismatrix(v) && numel(v) > best_n
            best   = v;
            best_n = numel(v);
        end
    end
    raw = best;
end

if isempty(raw)
    error('ody_load_data: data matrix is empty');
end

[Nt, Nvar] = size(raw);

% ---- collect column-index variables from workspace ----------------------
% These are uppercase names (M_DEPTH, C_SPEED, REAL_X, …)
col_map = struct();   % col_map.(varname_lower) = column_index
fnames  = fieldnames(ws);
var_names = cell(Nvar, 1);
for i = 1:numel(fnames)
    v = ws.(fnames{i});
    if isnumeric(v) && isscalar(v) && v >= 1 && v <= Nvar && floor(v)==v
        col_map.(lower(fnames{i})) = v;
        var_names{v} = lower(fnames{i});
    end
end

% ---- build output struct -------------------------------------------------
D.raw      = raw;
D.vars     = var_names;
D.Nt       = Nt;
D.Nvar     = Nvar;
D.dat_file = dat_file;
D.m_file   = m_file;
D.run_name = '';
if isfield(ws, 'run_name')
    D.run_name = ws.run_name;
end

% add per-variable fields
cnames = fieldnames(col_map);
for i = 1:numel(cnames)
    cn  = cnames{i};
    col = col_map.(cn);
    D.(cn) = raw(:, col);
end

% time vector
if isfield(D, 'm_present_time')
    D.t = D.m_present_time;
else
    D.t = (0:Nt-1)' * 0.02;   % fallback: 50 Hz
end

% ---- convenience groupings -----------------------------------------------
D.pos.north = get_or_nan(D, 'm_pos_n',  Nt);
D.pos.east  = get_or_nan(D, 'm_pos_e',  Nt);
D.pos.depth = get_or_nan(D, 'm_depth',  Nt);
D.pos.alt   = get_or_nan(D, 'm_altitude', Nt);

D.att.roll  = get_or_nan(D, 'm_roll',   Nt);
D.att.pitch = get_or_nan(D, 'm_pitch',  Nt);
D.att.yaw   = get_or_nan(D, 'm_yaw',    Nt);

D.ctrl.cmd_depth    = get_or_nan(D, 'c_depth',           Nt);
D.ctrl.cmd_heading  = get_or_nan(D, 'c_heading',         Nt);
D.ctrl.cmd_speed    = get_or_nan(D, 'c_speed',           Nt);
D.ctrl.cmd_elev     = get_or_nan(D, 'c_elevator_angle',  Nt);
D.ctrl.cmd_rud      = get_or_nan(D, 'c_rudder_angle',    Nt);
D.ctrl.cmd_thr      = get_or_nan(D, 'c_thruster_current',Nt);
D.ctrl.meas_elev    = get_or_nan(D, 'm_elevator_angle',  Nt);
D.ctrl.meas_rud     = get_or_nan(D, 'm_rudder_angle',    Nt);
D.ctrl.meas_thr     = get_or_nan(D, 'm_thruster_current',Nt);
D.ctrl.speed        = get_or_nan(D, 'm_water_speed',     Nt);

D.truth.x    = get_or_nan(D, 'real_x',     Nt);
D.truth.y    = get_or_nan(D, 'real_y',     Nt);
D.truth.z    = get_or_nan(D, 'real_z',     Nt);
D.truth.u    = get_or_nan(D, 'real_u',     Nt);
D.truth.phi  = get_or_nan(D, 'real_phi',   Nt);
D.truth.theta= get_or_nan(D, 'real_theta', Nt);
D.truth.psi  = get_or_nan(D, 'real_psi',  Nt);

end

% =========================================================================
%  Helpers
% =========================================================================

function v = get_or_nan(D, fname, Nt)
if isfield(D, fname)
    v = D.(fname);
else
    v = nan(Nt, 1);
end
end

function ws = run_m_file_in_workspace(m_file, dat_file)
% Run the .m index file and capture all resulting variables in struct ws.
%
% The parse-generated .m file expects to be run from the directory that
% contains the .dat file, and it calls load() by relative name.  We
% temporarily cd there.

[dat_dir, dat_base] = fileparts(dat_file);
old_dir = cd(dat_dir);

% Read the .m file and execute it here so we capture its workspace.
% We replace any 'review;' call (interactive review utility) with a no-op.
fid = fopen(m_file, 'r');
src = fread(fid, '*char')';
fclose(fid);

% Suppress interactive calls
src = regexprep(src, 'review\s*;', '% review;');
src = regexprep(src, 'clear\s+\S+\s*;?', '');  % drop "clear varname"

try
    eval(src);
catch ME
    cd(old_dir);
    error('ody_load_data: error running index .m file: %s', ME.message);
end
cd(old_dir);

% Capture everything that is now in the local workspace
ws = struct();
local_vars = who();
for i = 1:numel(local_vars)
    ws.(local_vars{i}) = eval(local_vars{i});
end
end
