function set_phi_expr(expr)
%SET_PHI_EXPR  Write a barrier function expression into fun_safety_condition.m
%   Replaces the line tagged with %#PHI_EXPR.
%   This file lives in codes/phi/; fun_safety_condition.m stays in codes/.

    thisDir = fileparts(mfilename('fullpath'));   % .../codes/phi
    codesDir = fileparts(thisDir);               % .../codes
    fpath = fullfile(codesDir, 'fun_safety_condition.m');
    txt = fileread(fpath);
    txt = regexprep(txt, '^\s*phi\s*=\s*.*%#PHI_EXPR', ...
        ['    phi = ' expr '; %#PHI_EXPR'], 'lineanchors');
    fid = fopen(fpath, 'w');
    fwrite(fid, txt);
    fclose(fid);
    clear fun_safety_condition;
    rehash;
end
