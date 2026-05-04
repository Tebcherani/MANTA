%  SIM_ALTITUDE - Altitude over bottom and depth profile.
%
%  The simulator's altimeter model produces m_altitude (range to bottom).
%  Without a real bathymetric map we reconstruct implied bottom depth as:
%       bottom_depth = m_depth + m_altitude
%  and plot that alongside the vehicle depth profile.  This reveals whether
%  the vehicle is tracking the bottom, maintaining a fixed altitude
%  setpoint, or flying blind above a feature.
%
%  Also plots depth rate (from fix_depth) and the yoyo classification.
%
%  Assumes on the workspace:
%    data        - Nt x Nvar data matrix (from parse)
%    time        - mission time vector (from fix_time)
%    run_name    - string label
%    d_rate      - depth rate (m/s), from fix_depth
%    yoyo        - yoyo index array, from fix_depth
%    going_up, going_down - from fix_depth
%    Column-index variables:
%      m_depth, m_altitude (altimeter range, metres)
%      c_depth             (commanded depth, optional)
%      real_alt            (true simulated altitude, optional truth check)
%
%    J. G. Bellingham / MIT Sea Grant AUV Lab
%    Adapted for simulator, 2025

global titleStr run_name

str = [run_name, titleStr];

have_alt      = exist('m_altitude')  == 1;
have_cmd      = exist('c_depth')     == 1;
have_real_alt = exist('real_alt')    == 1;

figure;
set( gcf, 'Name', ['Altitude: ', run_name] );
clf;

% -----------------------------------------------------------------------
%  Row 1: Depth profile + implied bottom depth + commanded depth
%         (depth axis inverted so deeper = lower on plot)
% -----------------------------------------------------------------------
subplot(3,1,1);

veh_depth = data(:, m_depth);

% Implied bottom depth from altimeter
if have_alt
    alt_meas = data(:, m_altitude);
    % Zero or negative altitude means no bottom lock; NaN those out.
    alt_meas( alt_meas <= 0 ) = NaN;
    bottom_depth = veh_depth + alt_meas;
    plot( time, bottom_depth, 'Color', [0.6 0.4 0.0], 'LineWidth', 1.2 );
    hold on;
end

% Vehicle depth (going up = red, going down = blue, like existing review.m)
abc = veh_depth;  abc(going_down) = NaN;
plot( time, abc, 'r', 'LineWidth', 1.2 );
hold on;
abc = veh_depth;  abc(going_up) = NaN;
plot( time, abc, 'b', 'LineWidth', 1.2 );

if have_cmd
    plot( time, data(:, c_depth), 'k--', 'LineWidth', 0.8 );
    leg = {'Implied bottom', 'Vehicle (up)', 'Vehicle (down)', 'Commanded'};
else
    leg = {'Implied bottom', 'Vehicle (up)', 'Vehicle (down)'};
end
if ~have_alt
    leg = leg(2:end);
end

max_bd = max(bottom_depth); if isnan(max_bd) || max_bd <= 0, max_bd = max(veh_depth)*1.5; end; if isnan(max_bd) || max_bd <= 0, max_bd = 100; end; set( gca, 'YDir', 'reverse', 'YLim', [0 max_bd*1.15] );
ylabel('Depth (m)');
title( ['Altitude / Depth Profile: ', str] );
legend( leg{:}, 'Location', 'best' );
grid on; zoom on;

% -----------------------------------------------------------------------
%  Row 2: Altitude over bottom (measured vs truth if available)
% -----------------------------------------------------------------------
subplot(3,1,2);

if have_alt
    plot( time, alt_meas, 'b', 'LineWidth', 1.2 );
    hold on;

    if have_real_alt
        plot( time, data(:, real_alt), 'k--', 'LineWidth', 0.8 );
        legend( 'Altimeter (noisy)', 'True altitude', 'Location', 'best' );
    end

    % Shade periods of no bottom lock in light red
    no_lock = isnan(alt_meas);
    if any(no_lock)
        yl = ylim;
        fill( [time(no_lock); flipud(time(no_lock))], ...
              [yl(1)*ones(sum(no_lock),1); yl(2)*ones(sum(no_lock),1)], ...
              'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none' );
        legend_append = {'No bottom lock'};
        if have_real_alt
            legend( 'Altimeter (noisy)', 'True altitude', legend_append{:}, ...
                    'Location', 'best' );
        else
            legend( 'Altimeter (meas)', legend_append{:}, 'Location', 'best' );
        end
    end

    ylabel('Altitude (m)');
    title('Altitude over bottom');
else
    text(0.5, 0.5, 'No altimeter data (m\_altitude not present)', ...
         'Units', 'normalized', 'HorizontalAlignment', 'center');
    title('Altitude over bottom — NO DATA');
end
grid on; zoom on;

% -----------------------------------------------------------------------
%  Row 3: Depth rate + yoyo classification
% -----------------------------------------------------------------------
subplot(3,1,3);

plot( time, d_rate, 'b', 'LineWidth', 1.0 );
hold on;
plot( [time(1) time(end)], [0 0], 'k-' );

% Shade each yoyo cycle alternately for easy counting
if exist('yoyo') == 1 && ~all(isnan(yoyo))
    n_yo = max(yoyo);
    colors = {[0.85 0.92 1.0], [1.0 0.92 0.85]};   % alternating pale blue/orange
    for k = 1:n_yo
        idx = find(yoyo == k);
        if isempty(idx), continue; end
        t0 = time(idx(1));  t1 = time(idx(end));
        yl = ylim;
        fill( [t0 t1 t1 t0], [yl(1) yl(1) yl(2) yl(2)], ...
              colors{mod(k,2)+1}, 'EdgeColor', 'none', 'FaceAlpha', 0.4 );
    end
    % Re-draw the depth rate line on top
    plot( time, d_rate, 'b', 'LineWidth', 1.0 );
end

% Overlay going_up / going_down markers
plot( time(going_up),   d_rate(going_up),   'r.', 'MarkerSize', 3 );
plot( time(going_down), d_rate(going_down), 'b.', 'MarkerSize', 3 );

ylabel( 'Depth rate (m/s)' );
xlabel( 'Mission time (s)' );
if exist('yoyo') == 1 && ~all(isnan(yoyo))
    title( sprintf('Depth rate and yoyo classification  (%d yoyos)', n_yo) );
else
    title( 'Depth rate' );
end
grid on; zoom on;