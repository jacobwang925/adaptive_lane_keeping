# Backwards compatibility notes

Temporary log while `function-prompting` diverges from `main`. Remove or fold into real docs after merge.

## `main_single_run.m`

Call sites in this repo use the function form and the `results` output; nothing still depends on the old script dumping variables into the base workspace.

| Caller | Status |
|--------|--------|
| `codes/phi/run_llm_pipeline.m` | OK — passes `opts`, uses return struct |
| `codes/phi/compare_phi_expressions.m` | OK — four args; `opts` defaults inside `main_single_run` |

**Not covered by repo code**

- `README.md` suggests `run('main_single_run.m')`. Prefer `main_single_run` or `results = main_single_run()` once the file is function-only.
- External scripts that assumed base-workspace side effects need to switch to `results`.

## Other branch deltas (BC-relevant)

- **`fun_safety_condition.m`**: default φ for three-arg calls matches `main`. **Optional `emax`:** if called with two arguments or `emax` is empty, **`emax` defaults to `5`** (same order of magnitude as the old commented default in `lane_keeping` and as `fun_inequality_direct_lane_keep`). Legacy **`mfun_safe_probability_mpc`** (six inputs) keeps using **`fun_safety_condition(x,xi)`** without a Simulink signature change; pass **`emax` explicitly** when it must match a scenario-specific bound. `set_phi_expr` / `%#PHI_EXPR` unchanged for tooling.
- **`fun_inequality_direct_lane_keep.m`**: aligned with `main`; anything documenting DIRECT + φ should match that (see README follow-up).
- **LLM JSON (`phi_expr`)**: in-repo callers updated; missing field is tolerated where validation allows.
- **`fun_mpc_controller.mexa64`**: tracked again after `.gitignore` change; clones without the binary still need a local build.

## Follow-up

- README: DIRECT / `fun_safety_condition` wording vs reverted direct inequality.
- Finish diff vs `main` file-by-file; commit any loose changes.
