"""
Assumptions:
- You run this from the folder that contains your participant files
  (PXX_CyclingModel_rra.osim, PXXrra_cycling_Kinematics_q.sto, etc.)
  OR you pass base_dir=... to the functions.

OpenSim requirement:
- OpenSim 4.x Python bindings with Moco enabled.
"""

from __future__ import annotations
from pathlib import Path
from typing import Iterable, List, Sequence, Optional, Union

import math
import csv
import opensim as osim


# -------------------------
# Constants
# -------------------------

MUSCLE_LIST: List[str] = [
    'addbrev_r', 'addlong_r', 'addmagDist_r', 'addmagIsch_r', 'addmagMid_r', 'addmagProx_r',
    'bflh140_r', 'bfsh140_r', 'edl_r', 'ehl_r', 'fdl_r', 'fhl_r',
    'gaslat140_r', 'gasmed_r', 'glmax1_r', 'glmax2_r', 'glmax3_r', 'glmed1_r',
    'glmed2_r', 'glmed3_r', 'glmin1_r', 'glmin2_r', 'glmin3_r', 'grac_r',
    'iliacus_r', 'perbrev_r', 'perlong_r', 'piri_r', 'psoas_r', 'recfem_r',
    'sart_r', 'semimem_r', 'semiten_r', 'soleus_r', 'tfl_r', 'tibant_r',
    'tibpost_r', 'vasint_r', 'vaslat140_r', 'vasmed_r', 'addbrev_l', 'addlong_l',
    'addmagDist_l', 'addmagIsch_l', 'addmagMid_l', 'addmagProx_l', 'bflh140_l', 'bfsh140_l',
    'edl_l', 'ehl_l', 'fdl_l', 'fhl_l', 'gaslat140_l', 'gasmed_l',
    'glmax1_l', 'glmax2_l', 'glmax3_l', 'glmed1_l', 'glmed2_l', 'glmed3_l',
    'glmin1_l', 'glmin2_l', 'glmin3_l', 'grac_l', 'iliacus_l', 'perbrev_l',
    'perlong_l', 'piri_l', 'psoas_l', 'recfem_l', 'sart_l', 'semimem_l',
    'semiten_l', 'soleus_l', 'tfl_l', 'tibant_l', 'tibpost_l', 'vasint_l',
    'vaslat140_l', 'vasmed_l',
]

# 1 = ignore tendon compliance; 0 = allow compliance.
TENDON_COMPLIANCE_FLAGS: List[int] = [1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# Coordinate actuators (residuals + reserves).
COORDINATE_NAMES: List[str] = [
    'pelvis_tx', 'pelvis_ty', 'pelvis_tz', 'pelvis_tilt', 'pelvis_list', 'pelvis_rotation',
    'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', 'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l',
    'lumbar_extension', 'lumbar_bending', 'lumbar_rotation', 'knee_angle_r',
]

COORDINATE_OPTIMAL_FORCES: List[float] = [
    100, 100, 100,   # pelvis translations
    100, 100, 100,   # pelvis rotations
    0.1, 0.1, 10,    # right hip
    0.1, 0.1, 10,    # left hip
    0.1, 0.1, 0.1,   # lumbar
    0.1              # knee_angle_r
]

JOINTS_TO_WELD: List[str] = ['mtp_r', 'mtp_l', 'subtalar_r', 'subtalar_l']


# -------------------------
# Small helpers
# -------------------------

def _as_path(base_dir: Optional[Union[str, Path]], filename: str) -> str:
    if base_dir is None:
        return filename
    return str(Path(base_dir) / filename)


def _vec_str(items: Sequence[str]):
    """
    Many OpenSim Python bindings accept a plain Python list[str] where
    the C++ API expects std::vector<std::string>.
    """
    return list(items)


def add_coordinate_actuator(model: "osim.Model", coord_name: str, optimal_force: float) -> None:
    coord_act = osim.CoordinateActuator()
    coord_act.setName(f"reserve_jointset_{coord_name}")
    coord_act.setCoordinate(model.updCoordinateSet().get(coord_name))
    coord_act.setOptimalForce(float(optimal_force))
    model.addForce(coord_act)


def set_tendon_compliance_from_flags(
    model: "osim.Model",
    muscle_names: Sequence[str] = MUSCLE_LIST,
    ignore_flags: Sequence[int] = TENDON_COMPLIANCE_FLAGS,
) -> None:
    """
    After ModOpIgnoreTendonCompliance() + ModOpReplaceMusclesWithDeGrooteFregly2016(),
    the muscles are DeGrooteFregly2016Muscle.
    We then toggle ignore_tendon_compliance per muscle using the provided flags.
    """
    if len(muscle_names) != len(ignore_flags):
        raise ValueError(
            f"muscle_names has {len(muscle_names)} items but ignore_flags has {len(ignore_flags)}."
        )

    for name, flag in zip(muscle_names, ignore_flags):
        muscle = model.getMuscles().get(name)
        dgf = osim.DeGrooteFregly2016Muscle.safeDownCast(muscle)
        dgf.set_ignore_tendon_compliance(bool(int(flag)))

# paper said 1SD (4.7+-1cm) should be added to OFL


def fix_soleus_optimal_fiber_length(model: "osim.Model", ratio: float = 1.21) -> None:
    """
    The soleus optimal fiber length (both legs).
    We do it by name ('soleus_r', 'soleus_l') to avoid index mistakes.
    """
    for sol_name in ("soleus_r", "soleus_l"):
        if model.getMuscles().contains(sol_name):
            m = osim.DeGrooteFregly2016Muscle.safeDownCast(
                model.getMuscles().get(sol_name))
            new_len = m.get_optimal_fiber_length() * float(ratio)
            # Both styles exist in the API across versions; try both.
            if hasattr(m, "setOptimalFiberLength"):
                m.setOptimalFiberLength(new_len)
            else:
                m.set_optimal_fiber_length(new_len)


def flatten_spatial_vec(spatial_vec) -> List[float]:
    """
    Convert a SimTK::SpatialVec (moment Vec3, force Vec3) into a flat list of 6 floats.
    """
    try:
        mom = spatial_vec.get(0)
        frc = spatial_vec.get(1)
    except Exception:
        mom = spatial_vec[0]
        frc = spatial_vec[1]

    def _vec3_to_list(v) -> List[float]:
        try:
            return [float(v.get(0)), float(v.get(1)), float(v.get(2))]
        except Exception:
            return [float(v[0]), float(v[1]), float(v[2])]

    return _vec3_to_list(mom) + _vec3_to_list(frc)


def l2_norm(vals: Sequence[float]) -> float:
    return float(math.sqrt(sum(float(x) ** 2 for x in vals)))


def write_csv_rows(path: Union[str, Path], header: Sequence[str], rows: Iterable[Sequence[float]]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(list(header))
        for r in rows:
            w.writerow([float(x) for x in r])
