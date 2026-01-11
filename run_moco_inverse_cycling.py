"""
run_moco_inverse_cycling.py

Runs all participants for J1 and/or J2.

Usage examples:
  # Run everything (J1 then J2) in current folder
  python run_moco_inverse_cycling.py --scheme both

  # Run only J2 for P03 and P04
  python run_moco_inverse_cycling.py --scheme J2 --participants P03 P04

  # Run using files in a different directory
  python run_moco_inverse_cycling.py --scheme J1 --base_dir "C:/path/..."
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Tuple, Optional, Sequence, Union

from solve_moco_inverse_cycling_j1 import solve_moco_inverse_cycling_j1
from solve_moco_inverse_cycling_j2 import solve_moco_inverse_cycling_j2


# Default weights
""" Try different values) on participants.
For each run, compare:

1-tracking RMSE (angles, markers, GRFs,.....)
2-whether excitations/controls look smooth and plausible
3-whether joint torques / residuals get worse """
EE_WEIGHT_DEFAULT = 2.0
FORCE_WEIGHT_DEFAULT = 1e-3

# Participant/time windows
# ('Pxx', in_time, last_time, '_Jx_Pxx_result')
CASES_J1: List[Tuple[str, float, float, str]] = [('P01', 120.63, 122.9, '_J1_P01_result'), ('P02', 120.34, 122.8, '_J1_P02_result'), ('P03', 120.45, 122.8, '_J1_P03_result'), ('P04', 120.5, 122.6, '_J1_P04_result'), ('P05', 120.45, 122.85, '_J1_P05_result'), ('P06', 120.52, 122.9, '_J1_P06_result'), ('P07', 120.2, 122.3, '_J1_P07_result'), (
    'P08', 120.5, 122.75, '_J1_P08_result'), ('P09', 120.09, 122.65, '_J1_P09_result'), ('P10', 120.35, 122.65, '_J1_P10_result'), ('P11', 120.0, 122.45, '_J1_P11_result'), ('P12', 120.67, 122.95, '_J1_P12_result'), ('P13', 120.4, 122.68, '_J1_P13_result'), ('P14', 120.35, 122.65, '_J1_P14_result'), ('P15', 120.55, 122.8, '_J1_P15_result'), ('P16', 120.63, 122.68, '_J1_P16_result')]
CASES_J2: List[Tuple[str, float, float, str]] = [('P01', 120.63, 122.9, '_J2_P01_result'), ('P02', 120.34, 122.8, '_J2_P02_result'), ('P03', 120.45, 122.8, '_J2_P03_result'), ('P04', 120.5, 122.6, '_J2_P04_result'), ('P05', 120.45, 122.85, '_J2_P05_result'), ('P06', 120.52, 122.9, '_J2_P06_result'), ('P07', 120.2, 122.3, '_J2_P07_result'), (
    'P08', 120.5, 122.75, '_J2_P08_result'), ('P09', 120.09, 122.65, '_J2_P09_result'), ('P10', 120.35, 122.65, '_J2_P10_result'), ('P11', 120.0, 122.45, '_J2_P11_result'), ('P12', 120.67, 122.95, '_J2_P12_result'), ('P13', 120.4, 122.68, '_J2_P13_result'), ('P14', 120.35, 122.65, '_J2_P14_result'), ('P15', 120.55, 122.8, '_J2_P15_result'), ('P16', 120.63, 122.68, '_J2_P16_result')]


def _filter_cases(cases: Sequence[Tuple[str, float, float, str]], keep: Optional[Sequence[str]]) -> List[Tuple[str, float, float, str]]:
    if not keep:
        return list(cases)
    keep_set = {k.strip() for k in keep}
    return [c for c in cases if c[0] in keep_set]


def main(
    scheme: str = "both",
    participants: Optional[Sequence[str]] = None,
    ee_weight: float = EE_WEIGHT_DEFAULT,
    force_weight: float = FORCE_WEIGHT_DEFAULT,
    base_dir: Optional[Union[str, Path]] = None,
) -> None:
    scheme = scheme.upper()
    if scheme not in {"J1", "J2", "BOTH"}:
        raise ValueError("scheme must be one of: J1, J2, both")

    if scheme in {"J1", "BOTH"}:
        for pxx, t1, t2, path_name in _filter_cases(CASES_J1, participants):
            solve_moco_inverse_cycling_j1(
                pxx, t1, t2, ee_weight, path_name, base_dir=base_dir)

    if scheme in {"J2", "BOTH"}:
        for pxx, t1, t2, path_name in _filter_cases(CASES_J2, participants):
            solve_moco_inverse_cycling_j2(
                pxx, t1, t2, ee_weight, force_weight, path_name, base_dir=base_dir)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--scheme", default="both", help="J1, J2, or both")
    ap.add_argument("--participants", nargs="*", default=None,
                    help="Subset, e.g., P01 P02")
    ap.add_argument("--ee_weight", type=float, default=EE_WEIGHT_DEFAULT)
    ap.add_argument("--force_weight", type=float, default=FORCE_WEIGHT_DEFAULT)
    ap.add_argument("--base_dir", default=None)
    args = ap.parse_args()

    main(
        scheme=args.scheme,
        participants=args.participants,
        ee_weight=args.ee_weight,
        force_weight=args.force_weight,
        base_dir=args.base_dir,
    )
