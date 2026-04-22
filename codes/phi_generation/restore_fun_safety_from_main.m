function restore_fun_safety_from_main()
%RESTORE_FUN_SAFETY_FROM_MAIN  Copy main-branch barrier implementation back into codes/
%   After phi experiments, fun_safety_condition.m may differ from the committed main
%   version. This restores codes/fun_safety_condition.m from the snapshot
%   fun_safety_condition_main.m in this directory (content matches main).
    thisDir = fileparts(mfilename('fullpath'));
    codesDir = fileparts(thisDir);
    snap = fullfile(thisDir, 'fun_safety_condition_main.m');
    dest = fullfile(codesDir, 'fun_safety_condition.m');
    copyfile(snap, dest, 'f');
    clear fun_safety_condition;
    rehash;
end
