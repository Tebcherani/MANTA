%  SIM_VPLANE - Simulator vertical plane review.
%
%  Plots commanded vs. actual for all vertical-plane variables:
%    depth, pitch, speed, elevator angle.
%  Roll is included as an uncontrolled diagnostic.
%
%  Assumes on the workspace:
%    data        - Nt x Nvar data matrix (from parse)
%    time        - mission time vector (seconds), from fix_time
%    run_name    - string label
%    d_rate      - depth rate (m/s), from fix_depth
%    going_up / going_down - index sets from fix_depth
%    Column-index variables: m_depth, m_pitch, m_roll, c_depth,
%      c_elevator_angle, m_elevator_angle, c_speed, m_water_speed
%
%    J. G. Bellingham / MIT Sea Grant AUV Lab
%    Adapted for simulator, 2025

global titleStr run_name

str = [run_name, titleStr];

figure;
set( gcf, 'Name', ['Vertical Plane: ', run_name] );
clf;

% -----------------------------------------------------------------------
%  Row 1: Depth (commanded vs actual)
% -----------------------------------------------------------------------
subplot(4,1,1);

if exist('c_depth') == 1
    plot( time, data(:, c_depth), 'r--', 'LineWidth', 1.0 );
    hold on;
end
plot( time, data(:, m_depth), 'b', 'LineWidth', 1.2 );
set( gca, 'YDir', 'reverse' );
ylabel( 'Depth (m)' );
title( ['Vertical Plane: ', str] );
if exist('c_depth') == 1
    legend( 'Commanded', 'Measured', 'Location', 'best' );
end
grid on; zoom on;

% -----------------------------------------------------------------------
%  Row 2: Pitch (commanded pitch comes from depth PID - show measured only)
%           Overlay commanded pitch limit (c_pitch) if present.
% -----------------------------------------------------------------------
subplot(4,1,2);

plot( time, data(:, m_pitch)/d2r, 'b', 'LineWidth', 1.2 );
hold on;
if exist('c_pitch') == 1
    plot( time, data(:, c_pitch)/d2r, 'r--', 'LineWidth', 1.0 );
    legend( 'Measured', 'Limit', 'Location', 'best' );
end
% Mark going-up / going-down
plot( time(going_up),   data(going_up,   m_pitch)/d2r, 'r.', 'MarkerSize', 2 );
plot( time(going_down), data(going_down, m_pitch)/d2r, 'b.', 'MarkerSize', 2 );
ylabel( 'Pitch (deg)' );
grid on; zoom on;

% -----------------------------------------------------------------------
%  Row 3: Elevator (commanded vs measured)
% -----------------------------------------------------------------------
subplot(4,1,3);

if exist('c_elevator_angle') == 1
    plot( time, data(:, c_elevator_angle)/d2r, 'r--', 'LineWidth', 1.0 );
    hold on;
end
if exist('m_elevator_angle') == 1
    plot( time, data(:, m_elevator_angle)/d2r, 'b', 'LineWidth', 1.2 );
end
ylabel( 'Elevator (deg)' );
if exist('c_elevator_angle') == 1 && exist('m_elevator_angle') == 1
    legend( 'Commanded', 'Measured', 'Location', 'best' );
end
grid on; zoom on;

% -----------------------------------------------------------------------
%  Row 4: Speed (commanded vs measured) + depth rate overlay
% -----------------------------------------------------------------------
subplot(4,1,4);

if exist('c_speed') == 1
    plot( time, data(:, c_speed), 'r--', 'LineWidth', 1.0 );
    hold on;
end
if exist('m_water_speed') == 1
    plot( time, data(:, m_water_speed), 'b', 'LineWidth', 1.2 );
    hold on;
end
% Depth rate on right axis gives intuition about vertical speed component
yyaxis right;
plot( time, d_rate, 'Color', [0 0.6 0], 'LineWidth', 0.8 );
ylabel( 'Depth rate (m/s)' );
yyaxis left;
ylabel( 'Speed (m/s)' );
xlabel( 'Mission time (s)' );
if exist('c_speed') == 1 && exist('m_water_speed') == 1
    legend( 'Cmd speed', 'Meas speed', 'Depth rate', 'Location', 'best' );
end
grid on; zoom on;
