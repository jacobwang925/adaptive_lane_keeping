function set_phi_expr(expr)
%SET_PHI_EXPR  Write a barrier function expression into fun_safety_condition.m
%   Replaces the line tagged with %#PHI_EXPR.

    fpath = fullfile(fileparts(mfilename('fullpath')), 'fun_safety_condition.m');
    txt = fileread(fpath);
    txt = regexprep(txt, '^\s*phi\s*=\s*.*%#PHI_EXPR', ...
        ['    phi = ' expr '; %#PHI_EXPR'], 'lineanchors');
    fid = fopen(fpath, 'w');
    fwrite(fid, txt);
    fclose(fid);
    clear fun_safety_condition;
    rehash;
end
