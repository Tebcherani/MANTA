%  SIM_HEADING - Simulator horizontal plane review.
%
%  Plots commanded vs. actual for all horizontal-plane variables:
%    heading (yaw), yaw rate, rudder angle.
%  Also shows cross-track error (WP_XTE) and distance-to-waypoint (WP_DTW)
%  when waypoint navigation is active.
%
%  Assumes on the workspace:
%    data     - Nt x Nvar data matrix (from parse)
%    time     - mission time vector (seconds), from fix_time
%    run_name - string label
%    Column-index variables: m_yaw, c_heading, m_yaw_rate,
%      c_rudder_angle, m_rudder_angle, m_roll
%      Optional: wp_xte, wp_dtw
%
%    J. G. Bellingham / MIT Sea Grant AUV Lab
%    Adapted for simulator, 2025

global titleStr run_name

str = [run_name, titleStr];

figure;
set( gcf, 'Name', ['Heading: ', run_name] );
clf;

% -----------------------------------------------------------------------
%  Row 1: Heading (commanded vs measured)
%  Note: wrap difference to [-180, 180] to avoid 360-deg jumps in cmd line.
% -----------------------------------------------------------------------
subplot(4,1,1);

meas_hdg = data(:, m_yaw) / d2r;
if exist('c_heading') == 1
    cmd_hdg = data(:, c_heading) / d2r;
    plot( time, cmd_hdg, 'r--', 'LineWidth', 1.0 );
    hold on;
    legend_str = {'Commanded', 'Measured'};
else
    legend_str = {'Measured'};
end
plot( time, meas_hdg, 'b', 'LineWidth', 1.2 );
ylabel( 'Heading (deg)' );
title( ['Horizontal Plane: ', str] );
legend( legend_str{:}, 'Location', 'best' );
grid on; zoom on;

% -----------------------------------------------------------------------
%  Row 2: Heading error (if commanded heading available)
% -----------------------------------------------------------------------
subplot(4,1,2);

if exist('c_heading') == 1
    err = meas_hdg - cmd_hdg;
    % Wrap to [-180, 180]
    err = mod(err + 180, 360) - 180;
    plot( time, err, 'k', 'LineWidth', 1.0 );
    hold on;
    plot( [time(1) time(end)], [0 0], 'r--' );
    ylabel( 'Heading error (deg)' );
    ylim( [-45 45] );
else
    % Fall back to yaw rate
    plot( time, data(:, m_yaw_rate)/d2r, 'b' );
    ylabel( 'Yaw rate (deg/s)' );
end
grid on; zoom on;

% -----------------------------------------------------------------------
%  Row 3: Rudder (commanded vs measured)
% -----------------------------------------------------------------------
subplot(4,1,3);

if exist('c_rudder_angle') == 1
    plot( time, data(:, c_rudder_angle)/d2r, 'r--', 'LineWidth', 1.0 );
    hold on;
end
if exist('m_rudder_angle') == 1
    plot( time, data(:, m_rudder_angle)/d2r, 'b', 'LineWidth', 1.2 );
end
ylabel( 'Rudder (deg)' );
if exist('c_rudder_angle') == 1 && exist('m_rudder_angle') == 1
    legend( 'Commanded', 'Measured', 'Location', 'best' );
end
grid on; zoom on;

% -----------------------------------------------------------------------
%  Row 4: Cross-track error and waypoint distance (or roll if no WP nav)
% -----------------------------------------------------------------------
subplot(4,1,4);

have_xte = exist('wp_xte') == 1;
have_dtw = exist('wp_dtw') == 1;

if have_xte
    plot( time, data(:, wp_xte), 'b', 'LineWidth', 1.2 );
    hold on;
    plot( [time(1) time(end)], [0 0], 'r--' );
    ylabel( 'Cross-track error (m)' );
    if have_dtw
        yyaxis right;
        plot( time, data(:, wp_dtw), 'Color', [0.5 0 0.5] );
        ylabel( 'Distance to WP (m)' );
    end
else
    % Roll is uncontrolled — useful to see if vehicle is banked
    plot( time, data(:, m_roll)/d2r, 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0 );
    hold on;
    plot( [time(1) time(end)], [0 0], 'r--' );
    ylabel( 'Roll (deg)  [uncontrolled]' );
    ylim( [-30 30] );
end
xlabel( 'Mission time (s)' );
grid on; zoom on;
