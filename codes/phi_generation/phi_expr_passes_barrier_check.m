function [ok, v0, ve, errMsg] = phi_expr_passes_barrier_check(expr_str, emax_check)
%PHI_EXPR_PASSES_BARRIER_CHECK  Shared phi validation for pipeline and phi ablation.
%   Parse expr as @(e,emax) phi with elementwise ops mapped to scalars (.*->*, ./->/, .^->^).
%   Pass iff:
%     (1) phi(0,emax) and phi(emax,emax) are real, v0 >= ~0, ve <= ~0 (see boundTol)
%     (2) phi(e,emax) is real, finite, and scalar for all e on a grid in [-2*emax, 2*emax]
%   Requirement (2) rejects forms like sqrt(1-(e/emax)^2): that expression is a real number
%   only when |e|<=emax; for |e|>emax it becomes complex / sqrt(domain error). MC/MPC often
%   samples |e|>emax, so phi must stay real-valued on [-2*emax, 2*emax].

v0 = NaN;
ve = NaN;
ok = false;
errMsg = "";

if nargin < 2 || isempty(emax_check) || ~isnumeric(emax_check) || ~isscalar(emax_check)
    errMsg = "emax_check must be a non-empty numeric scalar.";
    return;
end

if ~(emax_check > 0)
    errMsg = "emax_check must be positive.";
    return;
end

if isempty(expr_str)
    errMsg = "expr_str is empty.";
    return;
end

raw = string(expr_str);
if strlength(strtrim(raw)) == 0
    errMsg = "expr_str is empty.";
    return;
end

clean = strrep(strrep(strrep(raw, '.*', '*'), './', '/'), '.^', '^');
fn = [];
try
    fn = str2func("@(e, emax) " + clean);
    v0 = double(fn(0, emax_check));
    ve = double(fn(emax_check, emax_check));
catch ME
    v0 = NaN;
    ve = NaN;
    errMsg = string(ME.message);
    return;
end

% Boundary tolerances: expressions like cos(pi*e/(2*emax)) satisfy phi(emax)=0 in
% exact math, but cos(pi/2) in double precision is ~6e-17 (positive), so ve<=0
% would falsely reject them. Same for tiny float noise on phi(0)>=0.
boundTol = 1e-6;
if ~(isreal(v0) && isreal(ve) && isscalar(v0) && isscalar(ve) && ...
        isfinite(v0) && isfinite(ve) && v0 >= -boundTol && ve <= boundTol)
    return;
end

% Same e-range family as closed-loop / MC may see briefly beyond |e|<=emax
nGrid = 41;
eGrid = linspace(-2 * emax_check, 2 * emax_check, nGrid);
for ii = 1:nGrid
    try
        val = double(fn(eGrid(ii), emax_check));
    catch ME
        errMsg = string(ME.message);
        return;
    end
    if ~(isscalar(val) && isfinite(val) && isreal(val))
        errMsg = "phi must stay real and finite for e in [-2*emax, 2*emax] (e.g. avoid sqrt of negative when |e|>emax); failed at e=" ...
            + string(eGrid(ii)) + ", emax=" + string(emax_check);
        return;
    end
end

ok = true;
end
