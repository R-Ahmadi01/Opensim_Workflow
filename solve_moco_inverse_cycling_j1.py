"""
solve_moco_inverse_cycling_j1.py

Python port of solveMocoInverseCycling_J1.m
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Union

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


def solve_moco_inverse_cycling_j1(
    pxx: str,
    t_initial: float,
    t_final: float,
    ee_weight: float,
    path_name: str,
    base_dir: Optional[Union[str, Path]] = None,
) -> None:
    """
    Run MocoInverse for one participant using scheme J1.

    Parameters:
      solveMocoInverseCycling_J1(PXX, tInitial, tFinal, eeWeight, pathName)
    """
    # --- Files---
    model_in = _as_path(base_dir, f"{pxx}_CyclingModel_rra.osim")
    external_loads_xml = _as_path(base_dir, f"{pxx}external_loads_pedal.xml")
    kinematics_file = _as_path(base_dir, f"{pxx}rra_cycling_Kinematics_q.sto")
    guess_file = _as_path(
        base_dir, f"{pxx}Cycling_MocoInverse_solution_temp{path_name}.sto")
    states_rra_file = _as_path(base_dir, f"{pxx}rra_cycling_states.sto")

    solution_temp_out = _as_path(
        base_dir, f"{pxx}Cycling_MocoInverse_solution_temp{path_name}.sto")
    solution_out = _as_path(
        base_dir, f"{pxx}Cycling_MocoInverse_solution{path_name}.sto")
    states_out = _as_path(
        base_dir, f"{pxx}Cycling_MocoInverse_states{path_name}.sto")
    controls_out = _as_path(
        base_dir, f"{pxx}Cycling_MocoInverse_controls{path_name}.sto")
    model_out = _as_path(base_dir, f"{pxx}_CyclingModel_moco{path_name}.osim")
    forces_csv_out = _as_path(base_dir, f"{pxx}_ForceVectors{path_name}.csv")

    # --- Build ModelProcessor workflow  ---
    model_processor = osim.ModelProcessor(model_in)
    model_processor.append(osim.ModOpAddExternalLoads(external_loads_xml))
    model_processor.append(
        osim.ModOpReplaceJointsWithWelds(_vec_str(JOINTS_TO_WELD)))
    model_processor.append(osim.ModOpIgnoreTendonCompliance())
    model_processor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    model_processor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    try:
        model_processor.append(
            osim.ModOpTendonComplianceDynamicsModeDGF("implicit"))
    except Exception:
        # Older builds may use an enum or different signature.
        pass

    inverse = osim.MocoInverse()
    inverse.setName("cycling")
    inverse.setModel(model_processor)

    # Process model so we can tweak muscles and add coordinate actuators.
    model = model_processor.process()

    # Tendon compliance flags (per muscle).
    set_tendon_compliance_from_flags(
        model, MUSCLE_LIST, TENDON_COMPLIANCE_FLAGS)

    # Fix soleus warnings.
    fix_soleus_optimal_fiber_length(model, ratio=0.5)

    # Add residual / reserve actuators.
    for coord, opt_force in zip(COORDINATE_NAMES, COORDINATE_OPTIMAL_FORCES):
        add_coordinate_actuator(model, coord, opt_force)

    model.finalizeConnections()
    # Save processed model
    model.printToXML(model_out)

    # Update inverse with the final processed model.
    final_processor = osim.ModelProcessor(model)
    inverse.setModel(final_processor)

    # --- Inverse settings  ---
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

    # --- Initialize study and tune goal weights ---
    study = inverse.initialize()
    problem = study.updProblem()

    effort = osim.MocoControlGoal.safeDownCast(
        problem.updGoal("excitation_effort"))
    effort.setWeight(float(ee_weight))
    effort.setExponent(2)

    # --- Solver settings ---
    solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
    solver.resetProblem(problem)
    solver.set_optim_constraint_tolerance(1e-3)
    solver.set_optim_convergence_tolerance(1e-3)
    solver.set_num_mesh_intervals(100)
    solver.set_optim_max_iterations(1000)

    # --- Solve ---
    solution = study.solve()
    solution.write(solution_temp_out)

    # --- Insert RRA states into solution ---
    # Convert RRA states to radians for compatibility.
    rra_states_tp = osim.TableProcessor(states_rra_file)
    kinematics_needed = rra_states_tp.processAndConvertToRadians(model)

    # Create states trajectory and export to table.
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

    knee_r = osim.CustomJoint.safeDownCast(
        model.getJointSet().get("walker_knee_r"))
    knee_l = osim.CustomJoint.safeDownCast(
        model.getJointSet().get("walker_knee_l"))

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

        r_vec = flatten_spatial_vec(
            knee_r.calcReactionOnChildExpressedInGround(state))
        l_vec = flatten_spatial_vec(
            knee_l.calcReactionOnChildExpressedInGround(state))
        rows.append([state.getTime(), *r_vec, *l_vec,
                    l2_norm(r_vec), l2_norm(l_vec)])

    write_csv_rows(forces_csv_out, header, rows)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("pxx", help="Participant ID (e.g., P01)")
    ap.add_argument("t_initial", type=float)
    ap.add_argument("t_final", type=float)
    ap.add_argument("ee_weight", type=float)
    ap.add_argument("path_name", help="Suffix (e.g., _J1_P01_result)")
    ap.add_argument("--base_dir", default=None,
                    help="Folder containing participant files")
    args = ap.parse_args()

    solve_moco_inverse_cycling_j1(
        args.pxx, args.t_initial, args.t_final, args.ee_weight, args.path_name, base_dir=args.base_dir
    )
