function ody_write_mission(src_file, dst_file, overrides)
% ODY_WRITE_MISSION  Copy a mission file and apply parameter overrides.
%
%   ody_write_mission(src_file, dst_file, overrides)
%
% Reads src_file line-by-line. For each field in the overrides struct,
% any matching "PARAM value" line is replaced with the new value.
% Parameters not found in the source file are appended at the end
% (before any "state:" or "behavior:" sections, which must come last).
%
% INPUTS
%   src_file  : path to original mission / config file
%   dst_file  : path for the modified copy (may equal src_file to edit in place)
%   overrides : struct whose fieldnames are parameter names (case-insensitive)
%               and values are the new numeric or string values.
%
% EXAMPLE
%   overrides.UseElevator          = 0;
%   overrides.REAL_DEPTH_NOISE     = 0.5;
%   overrides.REAL_EXCESS_BUOYANCY = 3.0;
%   ody_write_mission('survey.mis', 'survey_fault.mis', overrides);

if nargin < 3 || isempty(overrides)
    copyfile(src_file, dst_file);
    return;
end

% read source
fid = fopen(src_file, 'r');
if fid < 0
    error('ody_write_mission: cannot open %s', src_file);
end
lines = {};
while ~feof(fid)
    lines{end+1} = fgetl(fid); %#ok<AGROW>
end
fclose(fid);

% build a map: lowercase param name → override value
fields   = fieldnames(overrides);
applied  = false(numel(fields), 1);

% index of first state/behavior line (overrides go before this)
first_state_line = numel(lines) + 1;
for i = 1:numel(lines)
    ln = strtrim(lines{i});
    if startsWith_ci(ln, 'state:') || startsWith_ci(ln, 'behavior:')
        first_state_line = i;
        break;
    end
end

% scan and replace
for i = 1:numel(lines)
    ln = lines{i};
    if ischar(ln)
        for fi = 1:numel(fields)
            fname = fields{fi};
            pat   = ['^', regexptranslate('escape', fname), '\s'];
            if ~isempty(regexpi(strtrim(ln), pat, 'once'))
                val = overrides.(fname);
                if isnumeric(val)
                    lines{i} = sprintf('%s %g', fname, val);
                else
                    lines{i} = sprintf('%s %s', fname, val);
                end
                applied(fi) = true;
                break;
            end
        end
    end
end

% append parameters that weren't found in the source
extra = {};
for fi = 1:numel(fields)
    if ~applied(fi)
        fname = fields{fi};
        val   = overrides.(fname);
        if isnumeric(val)
            extra{end+1} = sprintf('%s %g', fname, val); %#ok<AGROW>
        else
            extra{end+1} = sprintf('%s %s', fname, val); %#ok<AGROW>
        end
    end
end

if ~isempty(extra)
    % insert before first state/behavior line
    ins = first_state_line - 1;
    lines = [lines(1:ins), ...
             {'% --- injected overrides ---'}, ...
             extra(:)', ...
             lines(ins+1:end)];
end

% write output
fid = fopen(dst_file, 'w');
if fid < 0
    error('ody_write_mission: cannot write %s', dst_file);
end
for i = 1:numel(lines)
    ln = lines{i};
    if ischar(ln)
        fprintf(fid, '%s\n', ln);
    end
end
fclose(fid);

end

% -------------------------------------------------------------------------
function tf = startsWith_ci(str, prefix)
% case-insensitive startsWith
tf = strncmpi(str, prefix, numel(prefix));
end
