% FIX_TIME  Make mission time easily available.
%
% Creates 'time' as seconds elapsed since mission start.

time = data(:, m_present_time) - data(1, m_present_time);
