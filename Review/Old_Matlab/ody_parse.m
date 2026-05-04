function mfile = ody_parse(run_name)
% ODY_PARSE  Parse an Odyssey binary data file (.obd) — pure Matlab.
%            Replaces parse.exe, which was not included in the Windows build.
%
%   mfile = ody_parse()              % auto-selects newest .obd in SData\
%   mfile = ody_parse('S2609501')    % parse a specific run
%
% Reads the .obd binary format written by log_data.c and produces:
%   <run>.mat   data matrix + all sensor column-index variables
%   <run>.m     index script: loads the .mat and calls review
%
% BINARY FORMAT (from log_data.c / parse.c):
%   [ASCII HEADER]
%     Line 1:  MIT_SG_AUV_LAB_v2 <vehicle>
%     Line 2:  <datetime>  # OS-9 file creation time
%     Line 3:  <datetime>  # Structure creation time
%     Lines:   <idx> <sensor_name(units)> [initial_value]   one per sensor
%     Last:    9999 <mission_start_time> (<date>)
%   [BINARY SECTION]  -- flat stream of 6-byte pairs: int16 + float32
%     10001 | elapsed_time   <- frame sync (start of new control cycle)
%     <idx> | <value>        <- sensor update pairs until next 10001
%     ...
%
% BYTE ORDER: The Windows sim.exe writes native little-endian.
% The _M_I86 byte-swap in parse.c applied only to old 16-bit DOS builds.

cfg = ody_config();

% -------------------------------------------------------------------------
%  Locate the .obd file
% -------------------------------------------------------------------------
if nargin < 1 || isempty(run_name)
    hits = dir(fullfile(cfg.data_dir, '*.obd'));
    if isempty(hits)
        error('ody_parse: no .obd files found in %s', cfg.data_dir);
    end
    [~, newest] = max([hits.datenum]);
    obd_file = fullfile(cfg.data_dir, hits(newest).name);
    fprintf('ody_parse: auto-selected %s\n', hits(newest).name);
else
    [~, base, ext] = fileparts(run_name);
    if isempty(ext), ext = '.obd'; end
    obd_file = fullfile(cfg.data_dir, [base, ext]);
    if ~exist(obd_file, 'file')
        error('ody_parse: file not found: %s', obd_file);
    end
end

[data_dir, base_name] = fileparts(obd_file);
run_str  = lower(base_name);
mat_file = fullfile(data_dir, [run_str, '.mat']);
m_file   = fullfile(data_dir, [run_str, '.m']);

% -------------------------------------------------------------------------
%  Skip if already parsed and up to date
% -------------------------------------------------------------------------
obd_t = dir(obd_file);
if exist(mat_file,'file') && exist(m_file,'file')
    mat_t = dir(mat_file);
    if mat_t.datenum >= obd_t.datenum
        fprintf('ody_parse: %s already parsed, skipping.\n', base_name);
        mfile = m_file;
        return;
    end
end

fprintf('ody_parse: parsing %s ...\n', base_name);

% -------------------------------------------------------------------------
%  Read entire file as raw bytes
% -------------------------------------------------------------------------
fid = fopen(obd_file, 'rb');
if fid < 0, error('ody_parse: cannot open %s', obd_file); end
raw = fread(fid, inf, '*uint8');
fclose(fid);

% -------------------------------------------------------------------------
%  Parse ASCII header
%  Lines end with 0x0A (LF), optionally preceded by 0x0D (CR).
% -------------------------------------------------------------------------
lf_pos = find(raw == 10);

sensor_names   = {};
sensor_indices = [];
sensor_inits   = [];
binary_offset  = -1;
mission_start  = NaN;

byte_pos = 0;   % last consumed byte position (1-based end of previous line)

for li = 1:length(lf_pos)
    line_end   = lf_pos(li);
    line_bytes = raw(byte_pos+1 : line_end-1);
    line_bytes = line_bytes(line_bytes ~= 13);   % strip CR
    line       = strtrim(char(line_bytes'));
    byte_pos   = line_end;

    if isempty(line),            continue; end
    if any(line(1) == '#*%'),   continue; end   % comment lines
    if contains(line,'MIT_SG'), continue; end   % version line
    if contains(line, '#'),     continue; end   % date lines with inline #

    % Every remaining line starts with an integer
    [tok, rem] = strtok(line);
    idx_val = str2double(tok);
    if isnan(idx_val), continue; end
    idx_val = round(idx_val);

    % 9999 = mission start marker — final ASCII line
    if idx_val == 9999
        t_tok = strtok(strtrim(rem));
        mission_start = str2double(t_tok);
        binary_offset = line_end + 1;
        break;
    end

    % Sensor line:  idx   name(units)   [init_value]
    rem   = strtrim(rem);
    parts = strsplit(rem);
    if isempty(parts) || isempty(parts{1}), continue; end

    sname = parts{1};
    paren = strfind(sname, '(');
    if ~isempty(paren), sname = sname(1:paren(1)-1); end
    sname = lower(sname);

    init_val = NaN;
    if length(parts) >= 2
        v = str2double(parts{2});
        if ~isnan(v), init_val = v; end
    end

    sensor_names{end+1}   = sname;    %#ok<AGROW>
    sensor_indices(end+1) = idx_val;  %#ok<AGROW>
    sensor_inits(end+1)   = init_val; %#ok<AGROW>
end

% Fallback: if LF-based scan missed the 9999 line, search bytes directly.
% This handles files where set_filesize() pre-allocation puts extra LF bytes
% in the file, causing the line iterator to lose track of position.
if binary_offset < 0
    fprintf('ody_parse: LF scan missed 9999 line, trying direct byte search...\n');
    % Search for "9999 " as a byte pattern (ASCII 57 57 57 57 32)
    pat   = uint8('9999 ');
    found = false;
    for bi = 1:length(raw)-100
        if all(raw(bi:bi+4) == pat)
            % Verify preceded by LF, CR, or file start
            if bi == 1 || any(raw(bi-1) == [10 13 0])
                next_lf = find(raw(bi:bi+500) == 10, 1);
                if ~isempty(next_lf)
                    line_str = strtrim(char(raw(bi:bi+next_lf-2)'));
                    parts    = strsplit(line_str);
                    if length(parts) >= 2
                        mission_start = str2double(parts{2});
                    end
                    binary_offset = bi + next_lf;
                    fprintf('ody_parse: found 9999 at byte %d, binary at %d\n', ...
                            bi, binary_offset);
                    found = true;
                end
                break;
            end
        end
    end
    if ~found
        error('ody_parse: binary section not found (no 9999 line in header)');
    end
end

n_sensors = length(sensor_names);
max_idx   = max(sensor_indices);
fprintf('ody_parse: %d sensors, binary at byte %d\n', n_sensors, binary_offset);

% -------------------------------------------------------------------------
%  Parse binary section
%  Flat stream of 6-byte pairs: [int16 index][float32 value]
%  index == 10001 signals start of a new control cycle.
% -------------------------------------------------------------------------
bin     = uint8(raw(binary_offset:end));
n_pairs = floor(length(bin) / 6);
if n_pairs == 0
    error('ody_parse: no binary data found after header');
end
bin = bin(1 : n_pairs*6);

% Vectorised extraction — reshape into 6 × n_pairs, then typecast
chunks    = reshape(bin, 6, n_pairs);
idx_bytes = reshape(chunks(1:2,:), [], 1);   % 2*n_pairs x 1, column-major
val_bytes = reshape(chunks(3:6,:), [], 1);   % 4*n_pairs x 1, column-major
all_indices = typecast(uint8(idx_bytes), 'int16');
all_values  = typecast(uint8(val_bytes), 'single');

% -------------------------------------------------------------------------
%  Reconstruct time series
% -------------------------------------------------------------------------
FLAG      = int16(10001);
cycle_pos = find(all_indices == FLAG);
n_cycles  = length(cycle_pos);

if n_cycles == 0
    error('ody_parse: no cycle markers found — check byte order or file integrity');
end
fprintf('ody_parse: %d control cycles\n', n_cycles);

% Initialise first row from header initial values
n_cols   = max_idx + 1;
prev_row = nan(1, n_cols);
for si = 1:n_sensors
    if ~isnan(sensor_inits(si))
        prev_row(sensor_indices(si) + 1) = sensor_inits(si);
    end
end

data = nan(n_cycles, n_cols);
cycle_pos_aug = [cycle_pos; n_pairs + 1];  % sentinel for last cycle

for c = 1:n_cycles
    % Sensor pairs for this cycle follow the 10001 pair
    range = (cycle_pos(c) + 1) : (cycle_pos_aug(c+1) - 1);

    row = prev_row;   % carry forward all values from previous cycle

    if ~isempty(range)
        ci = double(all_indices(range));
        cv = double(all_values(range));
        ok = ci >= 0 & ci <= max_idx;
        row(ci(ok) + 1) = cv(ok);   % +1 converts 0-based C index to Matlab
    end

    data(c, :) = row;
    prev_row   = row;
end

% -------------------------------------------------------------------------
%  Build sensor-name → column-index map and save .mat
% -------------------------------------------------------------------------
col_vars = struct();
for si = 1:n_sensors
    vname = sensor_names{si};
    vname = regexprep(vname, '[^a-z0-9_]', '_');   % ensure valid identifier
    if isempty(vname) || ~isletter(vname(1))
        vname = ['s_', vname];
    end
    col_vars.(vname) = sensor_indices(si) + 1;   % 1-based column
end

save(mat_file, 'data', 'col_vars', 'mission_start', 'run_str');
fprintf('ody_parse: saved %s  [%d cycles x %d columns]\n', ...
        mat_file, n_cycles, n_cols);

% -------------------------------------------------------------------------
%  Write .m index file
% -------------------------------------------------------------------------
fid = fopen(m_file, 'w');
if fid < 0, error('ody_parse: cannot write %s', m_file); end

fprintf(fid, '%% Odyssey II data index — %s   %d cycles x %d sensors\n\n', ...
        run_str, n_cycles, n_sensors);
fprintf(fid, 'global run_name\n');
fprintf(fid, "run_name = '%s';\n\n", run_str);

fprintf(fid, '%% Sensor column indices into the data matrix\n');
fnames = fieldnames(col_vars);
for fi = 1:numel(fnames)
    fprintf(fid, '%-32s = %d;\n', fnames{fi}, col_vars.(fnames{fi}));
end

% Use full path to mat_file so load works regardless of cwd
fprintf(fid, "\nload('%s')  %% loads ''data'' matrix\n", ...
        strrep(mat_file, '\', '\\'));
fprintf(fid, 'review;\n');
fclose(fid);

fprintf('ody_parse: wrote %s\n', m_file);
mfile = m_file;
end

