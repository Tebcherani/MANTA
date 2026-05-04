%  SIM_REVIEW - Master mission review for the Odyssey simulator.
%
%  This is the simulator equivalent of the vehicle's review.m.  It calls
%  the four consolidated plot scripts that are meaningful for the sim:
%
%      sim_vplane    depth / pitch / elevator / speed vs commanded
%      sim_heading   heading / rudder vs commanded, cross-track error
%      sim_track     2-D and 3-D trajectory (DR + truth)
%      sim_altitude  altitude over bottom, depth profile, yoyo classification
%
%  USAGE
%  -----
%    % Basic (all four panels):
%    sim_review
%
%    % Select panels:
%    sim_opts = [1 3];  sim_review    % vplane + track only
%
%    % Suppress plots you don't want this session:
%    sim_opts = 'all';  sim_review
%
%  SWITCHES  (persist until cleared)
%    sim_opts   numeric array or 'all'.  Panel numbers:
%                 1 = sim_vplane
%                 2 = sim_heading
%                 3 = sim_track
%                 4 = sim_altitude
%
%  REQUIRES on the workspace before calling:
%    data, run_name   - loaded by the parse-generated .m file
%    units            - unit conversion constants (d2r, r2d, etc.)
%
%  SIM_REVIEW runs fix_time and fix_depth automatically.
%
%    J. G. Bellingham / MIT Sea Grant AUV Lab
%    Adapted for simulator, 2025

global titleStr run_name

% -----------------------------------------------------------------------
%  Sanity checks
% -----------------------------------------------------------------------
if exist('data') ~= 1
    disp('SIM_REVIEW: ''data'' matrix not found.  Load a parsed run first.');
    disp('  e.g.:  run(''mymission.m'')  % the parse-generated index file');
    return
end
if exist('run_name') ~= 1
    run_name = 'unknown';
end

% -----------------------------------------------------------------------
%  Unit conversions
% -----------------------------------------------------------------------
units;          % defines d2r, r2d, etc. — same script as real vehicle env

% -----------------------------------------------------------------------
%  Cosmetic: replace underscores in run_name for plot titles
% -----------------------------------------------------------------------
display_name = run_name;
jj = strfind(display_name, '_');
if ~isempty(jj)
    display_name(jj) = '.';
end

if ~exist('titleStr') || isempty(titleStr)
    titleStr = ['  [', display_name, ']'];
end

fprintf('\nSIM_REVIEW  run: %s\n', run_name);

% -----------------------------------------------------------------------
%  Pre-processing (mirrors review.m)
% -----------------------------------------------------------------------
fix_time;       % creates: time (relative to mission start)
fix_depth;      % creates: d_rate, yoyo, going_up, going_down

% -----------------------------------------------------------------------
%  Default panel selection
% -----------------------------------------------------------------------
if exist('sim_opts') ~= 1
    sim_opts = 'all';
end

% -----------------------------------------------------------------------
%  Plots
% -----------------------------------------------------------------------
do_all = strcmp(sim_opts, 'all');

%  1 — Vertical plane
if do_all || any(sim_opts == 1)
    sim_vplane;
    pause(0.3);
end

%  2 — Heading / horizontal plane
if do_all || any(sim_opts == 2)
    sim_heading;
    pause(0.3);
end

%  3 — Track (2-D + 3-D)
if do_all || any(sim_opts == 3)
    sim_track;
    pause(0.3);
end

%  4 — Altitude and depth profile
if do_all || any(sim_opts == 4)
    sim_altitude;
    pause(0.3);
end

fprintf('SIM_REVIEW done.\n\n');

% -----------------------------------------------------------------------
%  Cleanup (matches review.m style — don't leave run-specific flags behind)
% -----------------------------------------------------------------------
clear display_name jj do_all
