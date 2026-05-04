% FIX_DEPTH  Process depth: compute depth rate, classify yoyos.
%
%   1. Remove Paroscientific non-updates (every 10th cycle reads temperature
%      instead of pressure, leaving depth almost unchanged).
%   2. Compute depth rate by differentiation.
%   3. Classify yoyo cycles (down-up pairs deeper than 10 m).
%
% Requires: data, time, m_depth, m_present_time (from fix_time)
% Produces: d_rate, yoyo, going_up, going_down

d = data(:, m_depth);
n = length(d);
d(1:(n-1)) = d(2:n);

tmplt = d * NaN;
yoyo_discriminator = 0.1;

% Remove Paroscientific non-updates (unchanged depth every 10 cycles)
cyc  = find(abs(diff(d)) < 0.0015);
tens = find(abs(diff(cyc) - 10) < 0.003);
offset = mod(cyc(tens), 10);

if (std(offset) < 0.2) && (length(offset) > 10)
    d_off = round(mean(offset));
    bad   = 10*(0:(floor(length(d)/10)-1)) + d_off + 1;
    bad   = bad(bad <= length(d));
    d(bad) = NaN;
end

ok  = find(~isnan(d));

% Depth rate
d_r         = diff(d(ok)) ./ diff(time(ok));
d_r(length(ok)) = 0;
d_r         = smooth(d_r, 11);

d_rate      = tmplt;
d_rate(ok)  = d_r;

% Classify yoyos
ind10f = min(find(d > 10));
ind10l = max(find(d > 10));

if isempty(ind10f) || isempty(ind10l)
    yoyo       = tmplt;
    going_up   = find(d_rate < -yoyo_discriminator);
    going_down = find(d_rate >  yoyo_discriminator);
    clear d_r ok cyc tens offset;
    return;
end

earliest = max(find(d_rate(1:ind10f) < yoyo_discriminator));
tmp      = find(d_rate(ind10l:end)   > -yoyo_discriminator);
latest   = isempty(tmp) * length(d_rate) + ~isempty(tmp) * (tmp(1) + ind10l - 1);

ok1 = earliest:latest;

dwn  = find(diff(d_rate(ok1) > yoyo_discriminator) == 1);
okok = find(diff(dwn) > 100);
okok(2:(length(okok)+1)) = okok + 1;
okok(1) = 1;
if ~isempty(okok)
    dwn = ok1(dwn(okok));
end

yoyo = tmplt;
if length(dwn) > 2
    yoyo(1:dwn(1)) = 1;
    for i = 2:length(dwn)
        yoyo(dwn(i-1):dwn(i)) = i;
    end
    yoyo(dwn(end):end) = length(dwn) + 1;
end

going_up   = find(d_rate < -yoyo_discriminator);
going_down = find(d_rate >  yoyo_discriminator);

clear d_r ok okok dwn ok1 cyc tens offset earliest latest tmp ind10f ind10l;
