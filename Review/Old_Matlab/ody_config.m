function cfg = ody_config()
% ODY_CONFIG  Central configuration for the Odyssey simulator Matlab environment.
%
% Directory layout assumed:
%
%   AUV_Sim\
%     Windows_Ody_C\        <-- exe lives here; Mission\ and Data\ are
%       odyssey_sim.exe          junctions pointing to SMission\ and SData\
%       Mission\  -->  ..\SMission    (junction, created by setup_sim_config.bat)
%       Data\     -->  ..\SData       (junction, created by setup_sim_config.bat)
%     SMission\             <-- mission files (.mis)
%     SData\                <-- binary data files (.obd) and parsed output
%     EnvironmentModels\    <-- currents, bathymetry, ocean params
%     Review\               <-- this file + all Matlab scripts
%
% The exe MUST be launched from Windows_Ody_C\ (its own directory) because
% paths.h uses ODY_ROOT "." — the junctions make Mission\ and Data\
% resolve correctly from there.
%
% Once paths.h is updated and the exe recompiled, set the environment
% variable ODY_SIM_ROOT and the exe can be run from anywhere.

% =========================================================================
%  ROOT — everything else is relative to this
% =========================================================================

% cfg.root is the AUV_Sim directory.
% Derived from this file's location — works as long as ody_config.m
% lives in Review\ inside the AUV_Sim tree.
this_dir = fileparts(mfilename('fullpath'));   % …\AUV_Sim\Review
cfg.root = fileparts(this_dir);               % …\AUV_Sim

% =========================================================================
%  PATHS
% =========================================================================

cfg.exe_dir     = fullfile(cfg.root, 'Windows_Ody_C');
cfg.sim_exe     = fullfile(cfg.exe_dir, 'odyssey_sim.exe');

cfg.mission_dir = fullfile(cfg.root, 'SMission');
cfg.data_dir    = fullfile(cfg.root, 'SData');
cfg.env_dir     = fullfile(cfg.root, 'EnvironmentModels');
cfg.review_dir  = fullfile(cfg.root, 'Review');

% Quick sanity check — warn if the folders don't exist yet
for chk = {cfg.data_dir, cfg.mission_dir, cfg.exe_dir}
    if ~exist(chk{1}, 'dir')
        warning('ody_config: folder not found: %s', chk{1});
    end
end

% =========================================================================
%  SIMULATOR EXECUTION
% =========================================================================

% The exe is run via:   cd Windows_Ody_C && odyssey_sim.exe <mission_name>
% Mission name passed WITHOUT directory — the exe prepends .\Mission\
% which resolves through the junction to SMission\.
% Data files are written to .\Data\ which resolves through the junction
% to SData\.

cfg.sim_timeout_s = 300;   % wall-clock seconds before declaring sim hung

% MSYS2 bash — required to run sim.exe (compiled as MSYS2 binary)
cfg.msys2_bash = 'D:\ProgramsD\msys64\usr\bin\bash.exe';

cfg.parse_optset  = '';    % not used — parsing done in Matlab

% Parse utility — converts .obd binary to .dat + .m

% Data file naming: PREFIX_CHAR + YYYYY + run number + .obd  (from log_data.c)
% e.g. X25001_01.obd   (X = Xanthos prefix, 25 = year, 001 = day, 01 = run)
cfg.data_ext = '.obd';

% =========================================================================
%  SIMULATOR DEFAULTS
% =========================================================================

cfg.dt = 0.02;    % control loop time step (seconds)

% =========================================================================
%  FAULT LIBRARY
%  Struct of mission-file parameter overrides for each named fault.
%  Passed to ody_write_mission() before a run.
% =========================================================================

cfg.faults.nominal = struct();

cfg.faults.thruster_50pct = struct( ...
    'effective_drag_coeff', 0.42 );          % doubled drag ≈ 50% speed

cfg.faults.elevator_stuck = struct( ...
    'UseElevator', 0 );

cfg.faults.rudder_stuck = struct( ...
    'UseRudder', 0 );

cfg.faults.depth_noisy = struct( ...
    'REAL_DEPTH_NOISE', 1.0 );               % 1-sigma metres

cfg.faults.strong_current = struct( ...
    'REAL_CURRENT_NORTH', 0.5, ...
    'REAL_CURRENT_EAST',  0.3 );

cfg.faults.buoyancy_pos = struct( ...
    'REAL_EXCESS_BUOYANCY', 5.0 );           % Newtons

cfg.faults.altimeter_dropout = struct( ...
    'REAL_SONAR_FRAC_DROPOUT', 0.8 );

% =========================================================================
%  ENSEMBLE DEFAULTS
% =========================================================================

cfg.ensemble.n_runs    = 20;
cfg.ensemble.seed_base = 42;

end
