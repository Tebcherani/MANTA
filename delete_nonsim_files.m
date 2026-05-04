% delete_nonsim_files.m
%
% Deletes C source files, headers, and object files that are NOT needed
% for the standalone Odyssey simulator build.  These are hardware drivers,
% OS-9 daemons, and subsystems that are replaced by the simulator.
%
% SET dry_run = true FIRST to preview what will be deleted without
% actually deleting anything.  Then set dry_run = false to execute.
%
% Usage:
%   Run this script from any directory.  The target directory is hardcoded
%   in sim_dir below.

clear;

% -------------------------------------------------------------------------
% CONFIGURATION
% -------------------------------------------------------------------------
sim_dir = 'D:\OneDriveJHU\OneDrive - Johns Hopkins\Matlab-JHU\AUV_Mission_Sim\AUV_Sim\C_Sim';

dry_run = false;   % <-- set to false when you are ready to actually delete
% -------------------------------------------------------------------------

% Hardware sensor and actuator drivers
hardware_c = {
    'main.c'          % replaced by sim_main.c
    'io_utils.c'      % real serial-port I/O layer
    'serial.c'        % serial port driver
    'ody_serial.c'    % ODY serial abstraction
    'ody_serio.c'     % ODY serial I/O
    'kvh.c'           % KVH compass driver
    'sensor_kvh.c'    % KVH sensor processing
    'parosci.c'       % Paroscientific pressure sensor
    'fins.c'          % fin (elevator/rudder) actuator
    'thruster.c'      % thruster controller
    'sail.c'          % SAIL serial bus
    'rdi.c'           % RDI Workhorse ADCP
    'sontek.c'        % SonTek ADV
    'cond_temp.c'     % Sea-Bird SBE-25 CTD
    'sidescan.c'      % sidescan sonar
    'oa.c'            % obstacle-avoidance sonar driver
    'modem.c'         % acoustic modem driver
    'edc.c'           % EDC sensor
    'edc_homing.c'    % EDC homing
    'nrad.c'          % NRAD sensor
    'nrad_homing.c'   % NRAD homing
    'argos.c'         % ARGOS satellite transmitter
    'rf_modem.c'      % RF modem
    'camera.c'        % still camera
    'batcon.c'        % battery monitor
    'battest.c'       % battery test routine
    'remote_delay.c'  % timed remote command (hardware only)
    'icrc.c'          % CRC utility (hardware comms only)
    'whoi_latch.c'    % WHOI physical docking latch
};

% OS-9 daemons and inter-process communication
os9_c = {
    'missiond.c'      % OS-9 mission daemon
    'watchdog.c'      % OS-9 watchdog interface
    'talk.c'          % OS-9 inter-process comms
    'network.c'       % network layer (OS-9)
    'mcast.c'         % multicast (OS-9)
};

% Headers only used by the hardware files above
hardware_h = {
    'sail.h'          % SAIL bus
    'fins.h'          % fin actuator
    'cond_temp.h'     % CTD sensor
    'mcast.h'         % multicast
    'watchdog.h'      % watchdog
};

% Pre-compiled object files — stale, will be rebuilt
object_files = {
    'edc_homing.o'
    'fileman.o'
    'rdi.o'
    'sim_usbl.o'
    'whoi_traj.o'
};

% Combine all files into one list
all_files = [ hardware_c; os9_c; hardware_h; object_files ];

% -------------------------------------------------------------------------
% DRY RUN / EXECUTE
% -------------------------------------------------------------------------
if dry_run
    fprintf('\n*** DRY RUN — no files will be deleted ***\n');
    fprintf('Set dry_run = false to actually delete.\n\n');
else
    fprintf('\n*** DELETING FILES ***\n\n');
end

n_found    = 0;
n_missing  = 0;
n_deleted  = 0;
n_failed   = 0;

for i = 1:numel(all_files)
    fname = all_files{i};
    fpath = fullfile(sim_dir, fname);

    if isfile(fpath)
        n_found = n_found + 1;
        if dry_run
            fprintf('  [would delete] %s\n', fname);
        else
            try
                delete(fpath);
                fprintf('  [deleted]      %s\n', fname);
                n_deleted = n_deleted + 1;
            catch ME
                fprintf('  [FAILED]       %s  (%s)\n', fname, ME.message);
                n_failed = n_failed + 1;
            end
        end
    else
        n_missing = n_missing + 1;
        fprintf('  [not found]    %s\n', fname);
    end
end

% -------------------------------------------------------------------------
% SUMMARY
% -------------------------------------------------------------------------
fprintf('\n--- Summary ---\n');
fprintf('  Files found:    %d\n', n_found);
fprintf('  Files missing:  %d\n', n_missing);
if dry_run
    fprintf('  Would delete:   %d\n', n_found);
    fprintf('\nRe-run with dry_run = false to execute.\n');
else
    fprintf('  Deleted:        %d\n', n_deleted);
    if n_failed > 0
        fprintf('  Failed:         %d  <-- check file permissions\n', n_failed);
    end
end
fprintf('\n');
