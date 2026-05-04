%  SIM_TRACK - Simulator trajectory review.
%
%  Top panel:  2-D horizontal track.
%    - Dead-reckoned position (m_pos_n / m_pos_e) in blue.
%    - Simulator truth (real_x / real_y) in black dashed — the actual
%      body trajectory including currents and hydrodynamic drift.
%    - Waypoints extracted from commanded heading changes (approximate).
%
%  Bottom panel:  3-D trajectory (truth) with depth shown on the z-axis
%    and colormap indicating vehicle speed.
%
%  Assumes on the workspace:
%    data     - Nt x Nvar data matrix (from parse)
%    time     - mission time vector (from fix_time)
%    run_name - string label
%    Column-index variables:
%      m_pos_n, m_pos_e   (dead-reckoned position, metres)
%      real_x, real_y, real_z  (simulator truth body position)
%      m_water_speed      (speed m/s, used for colouring)
%      m_depth            (for 3-D z axis)
%      c_heading          (optional; marks heading-change waypoints)
%
%    J. G. Bellingham / MIT Sea Grant AUV Lab
%    Adapted for simulator, 2025

global titleStr run_name

str = [run_name, titleStr];

have_dr    = exist('m_pos_n') && exist('m_pos_e');
have_truth = exist('real_x')  && exist('real_y') && exist('real_z');
have_speed = exist('m_water_speed') == 1;

figure;
set( gcf, 'Name', ['Track: ', run_name] );
clf;

% -----------------------------------------------------------------------
%  Upper panel: 2-D horizontal track
% -----------------------------------------------------------------------
subplot(2,1,1);

legend_entries = {};

if have_truth
    tx = data(:, real_x);   % North in simulator coords
    ty = data(:, real_y);   % East
    plot( ty, tx, 'k--', 'LineWidth', 1.4 );
    hold on;
    legend_entries{end+1} = 'True track';
end

if have_dr
    dn = data(:, m_pos_n);
    de = data(:, m_pos_e);
    plot( de, dn, 'b', 'LineWidth', 1.0 );
    hold on;
    legend_entries{end+1} = 'Dead-reckoned';
end

% Mark start and end on whichever track is available
ref_n = []; ref_e = [];
if have_truth,    ref_n = tx;  ref_e = ty;
elseif have_dr,   ref_n = dn;  ref_e = de;
end
if ~isempty(ref_n)
    plot( ref_e(1),   ref_n(1),   'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g' );
    plot( ref_e(end), ref_n(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r' );
    legend_entries{end+1} = 'Start';
    legend_entries{end+1} = 'End';
end

xlabel('East (m)');
ylabel('North (m)');
title( ['2-D Track: ', str] );
legend( legend_entries{:}, 'Location', 'best' );
axis equal;
grid on;
zoom on;

% -----------------------------------------------------------------------
%  Lower panel: 3-D trajectory coloured by speed
% -----------------------------------------------------------------------
subplot(2,1,2);

if have_truth
    tx = data(:, real_x);
    ty = data(:, real_y);
    tz = -data(:, real_z);   % real_z is positive-down; flip for altitude feel

    if have_speed
        spd = data(:, m_water_speed);
        % Plot as coloured line segments
        n = length(tx);
        for k = 1:n-1
            seg_spd = 0.5*(spd(k) + spd(k+1));
            % Colour: blue (slow) → red (fast) based on 0–2 m/s range
            c = min(max(seg_spd / 2.0, 0), 1);
            clr = [c, 0, 1-c];
            plot3( ty(k:k+1), tx(k:k+1), tz(k:k+1), '-', ...
                   'Color', clr, 'LineWidth', 1.2 );
            hold on;
        end
        % Add a simple colorbar proxy
        cb_x = linspace(0, 2, 64);
        cb_c = [linspace(0,1,64)', zeros(64,1), linspace(1,0,64)'];
        % (no easy colourbar without a proper surface — label it manually)
        title( '3-D True Trajectory  (blue=slow, red=fast)' );
    else
        plot3( ty, tx, tz, 'k-', 'LineWidth', 1.2 );
        title( '3-D True Trajectory' );
    end

    plot3( ty(1),   tx(1),   tz(1),   'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g' );
    plot3( ty(end), tx(end), tz(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r' );

    xlabel('East (m)');
    ylabel('North (m)');
    zlabel('Depth (m, positive up)');
    set( gca, 'ZDir', 'normal' );
    grid on;
    view(30, 25);
    zoom on;

elseif have_dr
    % No truth: plot DR + depth
    dn = data(:, m_pos_n);
    de = data(:, m_pos_e);
    dz = -data(:, m_depth);
    plot3( de, dn, dz, 'b-', 'LineWidth', 1.2 );
    xlabel('East (m)');
    ylabel('North (m)');
    zlabel('Depth (m, positive up)');
    title( '3-D DR Trajectory (no truth available)' );
    grid on;
    view(30, 25);
else
    axis off;
    text(0.5, 0.5, 'Position data not available', ...
         'HorizontalAlignment', 'center');
end
