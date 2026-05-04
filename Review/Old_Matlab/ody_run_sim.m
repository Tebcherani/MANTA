function obd_file = ody_run_sim(mission_name)
% ODY_RUN_SIM  Run the Odyssey simulator from Matlab.
%
%   obd_file = ody_run_sim('dive240.mi')
%
% The recompiled sim.exe uses absolute paths (from paths.h), so it reads
% missions from SMission\ and writes data to SData\ regardless of where
% it is launched from.  No bash, no junctions, no file copying needed.

cfg = ody_config();

% ---- resolve mission file -----------------------------------------------
[~, mbase, mext] = fileparts(mission_name);
if isempty(mext), mext = '.mi'; end
src_mission = fullfile(cfg.mission_dir, [mbase, mext]);
if ~exist(src_mission, 'file')
    error('ody_run_sim: mission file not found: %s', src_mission);
end

% ---- note existing OBDs so we can spot the new one ---------------------
before = {dir(fullfile(cfg.data_dir, '*.obd')).name};

% ---- run sim from its own directory so get_drag.tex / get_am.tex resolve
old_dir = cd(cfg.exe_dir);
cmd = sprintf('"sim.exe" %s', [mbase, mext]);
fprintf('ody_run_sim: %s\n', [mbase, mext]);
[status, out] = system(cmd);
cd(old_dir);
if ~isempty(strtrim(out)), fprintf('%s\n', out); end
if status ~= 0
    warning('ody_run_sim: sim.exe exited with code %d', status);
end

% ---- find new OBD in SData\ ---------------------------------------------
after     = dir(fullfile(cfg.data_dir, '*.obd'));
new_files = after(~ismember({after.name}, before));

if isempty(new_files)
    error('ody_run_sim: no new .obd in %s\nCheck %s\nSData\system.log for errors.', ...
          cfg.data_dir, cfg.root);
end
[~, idx] = max([new_files.datenum]);
obd_file  = fullfile(cfg.data_dir, new_files(idx).name);
fprintf('ody_run_sim: done → %s\n', obd_file);

end
