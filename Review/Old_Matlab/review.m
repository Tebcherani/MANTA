% REVIEW  - Entry point for Odyssey simulator mission review.
%
% Works in two modes:
%
%   COLD START (no data loaded):
%     Typing "review" with an empty workspace calls ody_review(), which
%     finds the newest .obd in SData\, parses it, loads the data, and
%     plots.  This is the normal way to look at the most recent run.
%
%   CALLED BY PARSE-GENERATED .m FILE (data already loading):
%     The last line of every parse-generated index file is "review;" —
%     in that case 'data' is already in the workspace and we go straight
%     to sim_review.

if exist('data', 'var') ~= 1
    % No data in workspace — do the full parse + load + plot sequence.
    ody_review();
else
    % Data is already loaded (called from the parse-generated .m file).
    sim_review;
end
