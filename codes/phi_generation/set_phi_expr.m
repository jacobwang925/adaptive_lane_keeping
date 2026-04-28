function set_phi_expr(expr)
%SET_PHI_EXPR  Update the editable phi template and copy it into codes/fun_safety_condition.m
%   Edits the line tagged with %#PHI_EXPR in fun_safety_condition_phi.m (under codes/phi_generation/),
%   then copies that file to ../fun_safety_condition.m so Simulink/MPC use it.
%   For repository cleanliness, codes/ tracks main; restore with restore_fun_safety_from_main.

    thisDir = fileparts(mfilename('fullpath'));
    codesDir = fileparts(thisDir);
    templatePath = fullfile(thisDir, 'fun_safety_condition_phi.m');
    destPath = fullfile(codesDir, 'fun_safety_condition.m');

    txt = fileread(templatePath);
    txt = regexprep(txt, '^\s*phi\s*=\s*.*%#PHI_EXPR', ...
        ['    phi = ' expr '; %#PHI_EXPR'], 'lineanchors');
    fid = fopen(templatePath, 'w');
    fwrite(fid, txt);
    fclose(fid);

    copyfile(templatePath, destPath, 'f');

    clear fun_safety_condition;
    rehash;
end
