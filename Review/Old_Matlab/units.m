% UNITS  Unit conversion constants.
%
% Mirrors the units.m used in the real Odyssey vehicle environment.
% Call this script (not a function) so constants land in the caller's
% workspace.  sim_review calls it automatically.

d2r = pi / 180;      % degrees  → radians
r2d = 180 / pi;      % radians  → degrees

kts2ms  = 0.5144;    % knots    → m/s
ms2kts  = 1 / kts2ms;

ft2m    = 0.3048;    % feet     → metres
m2ft    = 1 / ft2m;

dbar2m  = 1.019716;  % decibar  → metres of seawater (approx)
