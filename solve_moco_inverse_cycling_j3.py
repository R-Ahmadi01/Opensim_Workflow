"""
solve_moco_inverse_cycling_j3.py

J3 = (J2 optional) + EMG tracking for a subset of muscles.

How it works:
  - We solve the same MocoInverse problem as J1/J2.
  - Then we add a MocoControlTrackingGoal that penalizes deviation between
    selected muscle excitations (controls) and your processed EMG envelopes.

Expected EMG file:
  - OpenSim .sto with a 'time' column and 1+ EMG columns.
  - EMG columns should be filtered/enveloped/normalized already (0..1).
  - Time must be in seconds and aligned with the kinematics time axis.

Mapping rule:
  - If your EMG column labels match muscle names in MUSCLE_LIST (e.g., 'gasmed_l'),
    you can omit emg_map and the script will auto-map:
        EMG column 'gasmed_l' -> control path '/forceset/gasmed_l'
  - Otherwise, provide emg_map explicitly.

Notes:
  - Moco tracks *controls* (muscle excitations) here (not activations).
  - Optional: enable scale factors (MocoParameters) to scale EMG up/down.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Union, Tuple

import re
import opensim as osim

from moco_cycling_common import (
    _as_path,
    _vec_str,
    MUSCLE_LIST,
    TENDON_COMPLIANCE_FLAGS,
    COORDINATE_NAMES,
    COORDINATE_OPTIMAL_FORCES,
    JOINTS_TO_WELD,
    add_coordinate_actuator,
    set_tendon_compliance_from_flags,
    fix_soleus_optimal_fiber_length,
    flatten_spatial_vec,
    l2_norm,
    write_csv_rows,
)


def _get_table_labels(table: "osim.TimeSeriesTable") -> list[str]:
    """Robustly convert OpenSim column labels to a Python list[str]."""
    try:
        # Some bindings allow iteration directly.
        return list(table.getColumnLabels())
    except Exception:
        labels = []
        try:
            n = table.getNumColumns()
        except Exception:
            n = table.getNumColumns()
        for i in range(int(n)):
            labels.append(str(table.getColumnLabel(int(i))))
        return labels


def _sanitize_param_name(s: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_]+", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    if not s:
        s = "emg"
    if s[0].isdigit():
        s = "p_" + s
    return s


def solve_moco_inverse_cycling_j3(
    pxx: str,
    t_initial: float,
    t_final: float,
    ee_weight: float,
    emg_weight: float,
    path_name: str,
    *,
    emg_file: Optional[Union[str, Path]] = None,
    emg_map: Optional[Dict[str, str]] = None,
    # If you want J2 behavior (joint reaction goal), set force_weight > 0.
    force_weight: float = 0.0,
    # Optional: let Moco optimize scale factors for each tracked EMG channel.
    enable_emg_scale_factors: bool = False,
    emg_scale_bounds: Tuple[float, float] = (0.01, 1.0),
    base_dir: Optional[Union[str, Path]] = None,
) -> None:
    """Run MocoInverse for one participant using scheme J3."""

    # --- Files (match MATLAB naming style) ---
    model_in = _as_path(base_dir, f"{pxx}_CyclingModel_rra.osim")
    external_loads_xml = _as_path(base_dir, f"{pxx}external_loads_pedal.xml")
    kinematics_file = _as_path(base_dir, f"{pxx}rra_cycling_Kinematics_q.sto")
    guess_file = _as_path(base_dir, f"{pxx}Cycling_MocoInverse_solution_temp{path_name}.sto")
    states_rra_file = _as_path(base_dir, f"{pxx}rra_cycling_states.sto")

    # Default EMG file name (edit if your naming differs).
    if emg_file is None:
        emg_file = _as_path(base_dir, f"{pxx}emg_normalized.sto")
    else:
        emg_file = _as_path(base_dir, str(emg_file))

    solution_temp_out = _as_path(base_dir, f"{pxx}Cycling_MocoInverse_solution_temp{path_name}.sto")
    solution_out = _as_path(base_dir, f"{pxx}Cycling_MocoInverse_solution{path_name}.sto")
    states_out = _as_path(base_dir, f"{pxx}Cycling_MocoInverse_states{path_name}.sto")
    controls_out = _as_path(base_dir, f"{pxx}Cycling_MocoInverse_controls{path_name}.sto")
    model_out = _as_path(base_dir, f"{pxx}_CyclingModel_moco{path_name}.osim")
    forces_csv_out = _as_path(base_dir, f"{pxx}_ForceVectors{path_name}.csv")

    # --- ModelProcessor workflow ---
    model_processor = osim.ModelProcessor(model_in)
    model_processor.append(osim.ModOpAddExternalLoads(external_loads_xml))
    model_processor.append(osim.ModOpReplaceJointsWithWelds(_vec_str(JOINTS_TO_WELD)))
    model_processor.append(osim.ModOpIgnoreTendonCompliance())
    model_processor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    model_processor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    try:
        model_processor.append(osim.ModOpTendonComplianceDynamicsModeDGF("implicit"))
    except Exception:
        pass

    inverse = osim.MocoInverse()
    inverse.setName("cycling")
    inverse.setModel(model_processor)

    # Process model so we can tweak muscles and add coordinate actuators.
    model = model_processor.process()

    # Tendon compliance flags (per muscle).
    set_tendon_compliance_from_flags(model, MUSCLE_LIST, TENDON_COMPLIANCE_FLAGS)

    # Fix soleus warnings.
    fix_soleus_optimal_fiber_length(model, ratio=0.5)

    # Add residual / reserve actuators.
    for coord, opt_force in zip(COORDINATE_NAMES, COORDINATE_OPTIMAL_FORCES):
        add_coordinate_actuator(model, coord, opt_force)

    model.finalizeConnections()
    model.printToXML(model_out)

    # Update inverse with the final processed model.
    final_processor = osim.ModelProcessor(model)
    inverse.setModel(final_processor)

    # --- Inverse settings ---
    inverse.setKinematics(osim.TableProcessor(kinematics_file))
    inverse.set_kinematics_allow_extra_columns(True)
    inverse.set_kinematics_throw_if_missing_columns(False)
    inverse.set_kinematics_lowpass_cutoff_frequency(6)
    inverse.set_initial_time(float(t_initial))
    inverse.set_final_time(float(t_final))
    inverse.set_mesh_interval(0.05)
    inverse.set_constrain_initial_configuration(False)
    inverse.set_constrain_final_configuration(False)
    inverse.set_minimize_implicit_auxiliary_derivatives(True)
    inverse.set_bound_activation_from_excitation(True)
    inverse.setGuessFile(guess_file)

    # --- Initialize and edit goals ---
    study = inverse.initialize()
    problem = study.updProblem()

    # Effort cost weight.
    effort = osim.MocoControlGoal.safeDownCast(problem.updGoal("excitation_effort"))
    effort.setWeight(float(ee_weight))
    effort.setExponent(2)

    # Optional J2 term: knee joint reaction goal.
    if float(force_weight) > 0:
        jr_goal = osim.MocoJointReactionGoal("knee_jr_cost")
        jr_goal.setJointPath("/jointset/walker_knee_r")
        jr_goal.setLoadsFrame("child")
        jr_goal.setExpressedInFramePath("/bodyset/tibia_r")
        jr_goal.setReactionMeasures(_vec_str(["force-y", "force-z"]))
        jr_goal.setWeight(float(force_weight))
        problem.addGoal(jr_goal)

    # --- EMG tracking (J3) ---
    tracking = osim.MocoControlTrackingGoal("emg_tracking")
    tracking.setWeight(float(emg_weight))
    # Different OpenSim builds support TableProcessor(str) and/or TableProcessor(TimeSeriesTable).
    try:
        tracking.setReference(osim.TableProcessor(emg_file))
    except Exception:
        tracking.setReference(osim.TableProcessor(osim.TimeSeriesTable(emg_file)))

    # Auto-map: EMG column labels that match a model muscle name.
    if emg_map is None:
        emg_tbl = osim.TimeSeriesTable(emg_file)
        labels = _get_table_labels(emg_tbl)
        muscle_set = set(MUSCLE_LIST)
        emg_map = {
            f"/forceset/{lab}": lab
            for lab in labels
            if lab in muscle_set
        }

    if not emg_map:
        raise RuntimeError(
            "No EMG channels were mapped. Either:\n"
            "  (1) make EMG column labels match muscle names (e.g., gasmed_l), or\n"
            "  (2) pass emg_map={'/forceset/gasmed_l':'your_emg_col', ...}."
        )

    # Map each tracked control to its EMG column.
    for control_path, emg_col in emg_map.items():
        tracking.setReferenceLabel(str(control_path), str(emg_col))

    # Optional: allow Moco to scale each EMG channel (helpful if your "normalized"
    # envelope does not match the excitation magnitude well).
    if enable_emg_scale_factors:
        lo, hi = float(emg_scale_bounds[0]), float(emg_scale_bounds[1])
        for control_path, emg_col in emg_map.items():
            pname = f"emg_sf_{_sanitize_param_name(str(emg_col))}"
            tracking.addScaleFactor(pname, str(control_path), [lo, hi])

    problem.addGoal(tracking)

    # --- Solver settings ---
    solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
    solver.resetProblem(problem)
    solver.set_optim_constraint_tolerance(1e-3)
    solver.set_optim_convergence_tolerance(1e-3)
    solver.set_num_mesh_intervals(100)
    solver.set_optim_max_iterations(1000)

    if enable_emg_scale_factors and hasattr(solver, "set_parameters_require_initsystem"):
        solver.set_parameters_require_initsystem(False)

    # --- Solve ---
    solution = study.solve()
    solution.write(solution_temp_out)

    # --- Insert RRA states into solution (like MATLAB) ---
    rra_states_tp = osim.TableProcessor(states_rra_file)
    kinematics_needed = rra_states_tp.processAndConvertToRadians(model)
    states_traj = osim.StatesTrajectory.createFromStatesTable(
        model, osim.TimeSeriesTable(kinematics_needed), True, True, False
    )
    kinematics_output = states_traj.exportToTable(model)

    sol = solution.unseal()
    sol.insertStatesTrajectory(kinematics_output, False)
    sol.write(solution_out)

    osim.STOFileAdapter.write(sol.exportToStatesTable(), states_out)
    osim.STOFileAdapter.write(sol.exportToControlsTable(), controls_out)

    # --- Joint reaction forces (walker_knee_r / walker_knee_l) ---
    model.initSystem()
    traj = sol.exportToStatesTrajectory(model)
    knee_r = osim.CustomJoint.safeDownCast(model.getJointSet().get("walker_knee_r"))
    knee_l = osim.CustomJoint.safeDownCast(model.getJointSet().get("walker_knee_l"))

    header = [
        "time",
        "right_Mx", "right_My", "right_Mz", "right_Fx", "right_Fy", "right_Fz",
        "left_Mx",  "left_My",  "left_Mz",  "left_Fx",  "left_Fy",  "left_Fz",
        "jointReactionNorm_right", "jointReactionNorm_left",
    ]
    rows = []
    for i in range(traj.getSize()):
        state = traj.get(i)
        model.realizePosition(state)
        r_vec = flatten_spatial_vec(knee_r.calcReactionOnChildExpressedInGround(state))
        l_vec = flatten_spatial_vec(knee_l.calcReactionOnChildExpressedInGround(state))
        rows.append([state.getTime(), *r_vec, *l_vec, l2_norm(r_vec), l2_norm(l_vec)])

    write_csv_rows(forces_csv_out, header, rows)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("pxx", help="Participant ID (e.g., P01)")
    ap.add_argument("t_initial", type=float)
    ap.add_argument("t_final", type=float)
    ap.add_argument("ee_weight", type=float)
    ap.add_argument("emg_weight", type=float)
    ap.add_argument("path_name", help="Suffix (e.g., _J3_P01_result)")
    ap.add_argument("--emg_file", default=None, help="EMG .sto file (default: {PXX}emg_normalized.sto)")
    ap.add_argument("--force_weight", type=float, default=0.0, help="If >0, also add J2 knee JRF cost")
    ap.add_argument("--enable_emg_scale_factors", action="store_true")
    ap.add_argument("--emg_scale_lo", type=float, default=0.01)
    ap.add_argument("--emg_scale_hi", type=float, default=1.0)
    ap.add_argument(
        "--emg_map",
        nargs="*",
        default=None,
        help=(
            "Optional explicit mapping items like '/forceset/gasmed_l:gasmed_l' or '/forceset/gasmed_l:gastrocnemius'. "
            "If omitted, auto-map EMG columns that match muscle names."
        ),
    )
    ap.add_argument("--base_dir", default=None)
    args = ap.parse_args()

    emg_map = None
    if args.emg_map:
        emg_map = {}
        for item in args.emg_map:
            if ":" not in item:
                raise ValueError(f"Bad --emg_map item (missing ':'): {item}")
            k, v = item.split(":", 1)
            emg_map[k.strip()] = v.strip()

    solve_moco_inverse_cycling_j3(
        args.pxx,
        args.t_initial,
        args.t_final,
        args.ee_weight,
        args.emg_weight,
        args.path_name,
        emg_file=args.emg_file,
        emg_map=emg_map,
        force_weight=args.force_weight,
        enable_emg_scale_factors=args.enable_emg_scale_factors,
        emg_scale_bounds=(args.emg_scale_lo, args.emg_scale_hi),
        base_dir=args.base_dir,
    )
    solve_moco_inverse_cycling_j3(
        args.pxx,
        args.t_initial,
        args.t_final,
        args.ee_weight,
        args.emg_weight,
        args.path_name,
        force_weight=args.force_weight,
        emg_map=emg_map,
        base_dir=args.base_dir,
    )