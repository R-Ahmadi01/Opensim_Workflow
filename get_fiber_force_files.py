# get_fiber_force_files.py
from __future__ import annotations
import argparse
from pathlib import Path
from typing import Callable, Dict, List
import opensim as osim


# Outputs + file naming
OUTPUTS: Dict[str, str] = {
    # output_file_prefix            "what to compute"
    "active_fiber_force_along_tendon":  "active_fiber_force_along_tendon",
    "passive_fiber_force_along_tendon": "passive_fiber_force_along_tendon",
    "fiber_force_along_tendon":         "fiber_force_along_tendon",
    "active_force_length_multiplier":   "active_force_length_multiplier",
    "force_velocity_multiplier":        "force_velocity_multiplier",
    "passive_force_multiplier":         "passive_force_multiplier",
}


def _find_muscles(model: osim.Model) -> List[osim.Muscle]:
    """Return all Muscle objects in the model's ForceSet."""
    muscles: List[osim.Muscle] = []
    forces = model.getForceSet()
    for i in range(forces.getSize()):
        f = forces.get(i)
        try:
            m = osim.Muscle.safeDownCast(f)
            if m is not None:
                muscles.append(m)
        except Exception:
            # If safeDownCast isn't available for some reason, skip.
            pass
    if not muscles:
        raise RuntimeError("No muscles found in the model ForceSet().")
    return muscles


""" kind can be <"active_fiber_force_along_tendon">,<"passive_fiber_force_along_tendon">,<"fiber_force_along_tendon">,
<"active_force_length_multiplier">,<"force_velocity_multiplier">,<"passive_force_multiplier"> """


def _get_value_getter(kind: str) -> Callable[[osim.Muscle, osim.State], float]:
    """
    Return a function that computes the requested quantity from a Muscle and State.
    """
    # Along-tendon forces
    if kind == "active_fiber_force_along_tendon":
        def f(m: osim.Muscle, s: osim.State) -> float:
            if hasattr(m, "getActiveFiberForceAlongTendon"):
                return float(m.getActiveFiberForceAlongTendon(s))
            if hasattr(m, "active_fiber_force_along_tendon"):
                return float(m.active_fiber_force_along_tendon(s))
            raise AttributeError("No API for active_fiber_force_along_tendon")
        return f

    if kind == "passive_fiber_force_along_tendon":
        def f(m: osim.Muscle, s: osim.State) -> float:
            if hasattr(m, "getPassiveFiberForceAlongTendon"):
                return float(m.getPassiveFiberForceAlongTendon(s))
            if hasattr(m, "passive_fiber_force_along_tendon"):
                return float(m.passive_fiber_force_along_tendon(s))
            raise AttributeError("No API for passive_fiber_force_along_tendon")
        return f

    if kind == "fiber_force_along_tendon":
        def f(m: osim.Muscle, s: osim.State) -> float:
            if hasattr(m, "getFiberForceAlongTendon"):
                return float(m.getFiberForceAlongTendon(s))
            if hasattr(m, "fiber_force_along_tendon"):
                return float(m.fiber_force_along_tendon(s))
            raise AttributeError("No API for fiber_force_along_tendon")
        return f

    # Multipliers
    if kind == "active_force_length_multiplier":
        def f(m: osim.Muscle, s: osim.State) -> float:
            if hasattr(m, "getActiveForceLengthMultiplier"):
                return float(m.getActiveForceLengthMultiplier(s))
            raise AttributeError("No API for active_force_length_multiplier")
        return f

    if kind == "force_velocity_multiplier":
        def f(m: osim.Muscle, s: osim.State) -> float:
            if hasattr(m, "getForceVelocityMultiplier"):
                return float(m.getForceVelocityMultiplier(s))
            raise AttributeError("No API for force_velocity_multiplier")
        return f

    if kind == "passive_force_multiplier":
        def f(m: osim.Muscle, s: osim.State) -> float:
            if hasattr(m, "getPassiveForceMultiplier"):
                return float(m.getPassiveForceMultiplier(s))
            raise AttributeError("No API for passive_force_multiplier")
        return f

    raise ValueError(f"Unknown output kind: {kind}")


def _std_vector_string(strings: List[str]) -> "osim.StdVectorString":
    v = osim.StdVectorString()
    for s in strings:
        v.append(s)
    return v


def _make_table(
    model: osim.Model,
    states_traj: osim.StatesTrajectory,
    muscles: List[osim.Muscle],
    getter: Callable[[osim.Muscle, osim.State], float],
) -> osim.TimeSeriesTable:
    labels = [m.getName() for m in muscles]

    table = osim.TimeSeriesTable()
    table.setColumnLabels(_std_vector_string(labels))

    # Determine how many States (time points / frames) are stored in the trajectory.
    n = states_traj.getSize() if hasattr(states_traj, "getSize") else len(states_traj)

    for i in range(n):
        state = states_traj[i]
        model.realizeDynamics(state)

        row = osim.RowVector(len(muscles))
        for j, mus in enumerate(muscles):
            row[j] = getter(mus, state)

        table.appendRow(state.getTime(), row)

    return table


def get_fiber_force_files(pxx: str, suffix: str, base_dir: str | Path) -> None:
    base_dir = Path(base_dir)

    model_path = base_dir / f"{pxx}_CyclingModel_moco{suffix}.osim"
    sol_path = base_dir / f"{pxx}Cycling_MocoInverse_solution{suffix}.sto"

    if not model_path.exists():
        raise FileNotFoundError(f"Model not found:\n  {model_path}")
    if not sol_path.exists():
        raise FileNotFoundError(f"Moco solution not found:\n  {sol_path}")

    model = osim.Model(str(model_path))
    model.initSystem()

    traj = osim.MocoTrajectory(str(sol_path))
    states_table = traj.exportToStatesTable()
    states_traj = osim.StatesTrajectory.createFromStatesTable(
        model, states_table)

    muscles = _find_muscles(model)

    for out_prefix, kind in OUTPUTS.items():
        getter = _get_value_getter(kind)
        table = _make_table(model, states_traj, muscles, getter)

        # matches your screenshot
        out_file = base_dir / f"{out_prefix}{suffix}.sto"
        osim.STOFileAdapter.write(table, str(out_file))
        print(f"[ok] wrote: {out_file}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Compute muscle fiber-force-along-tendon and multipliers from a MocoInverse solution."
    )
    ap.add_argument("pxx", help="Participant ID (e.g., P11)")
    ap.add_argument("suffix", help="Suffix (e.g., _J1_P11_result)")
    ap.add_argument(
        "base_dir", help="Folder containing the .osim and solution .sto files")
    args = ap.parse_args()

    get_fiber_force_files(args.pxx, args.suffix, args.base_dir)
