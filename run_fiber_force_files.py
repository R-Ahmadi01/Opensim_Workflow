"""
run_fiber_force_files.py

Python port of runFiberForceFilesFunction.m
Runs get_fiber_force_files.py for all participants for J1 and/or J2.

Usage:
  python run_fiber_force_files.py --scheme both
  python run_fiber_force_files.py --scheme J1 --participants P01 P02
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Tuple, Optional, Sequence, Union

from get_fiber_force_files import get_fiber_force_files

CASES_J1: List[Tuple[str, str]] = [('P01', '_J1_P01_result'), ('P02', '_J1_P02_result'), ('P03', '_J1_P03_result'), ('P04', '_J1_P04_result'), ('P05', '_J1_P05_result'), ('P06', '_J1_P06_result'), ('P07', '_J1_P07_result'), (
    'P08', '_J1_P08_result'), ('P09', '_J1_P09_result'), ('P10', '_J1_P10_result'), ('P11', '_J1_P11_result'), ('P12', '_J1_P12_result'), ('P13', '_J1_P13_result'), ('P14', '_J1_P14_result'), ('P15', '_J1_P15_result'), ('P16', '_J1_P16_result')]
CASES_J2: List[Tuple[str, str]] = [('P01', '_J2_P01_result'), ('P02', '_J2_P02_result'), ('P03', '_J2_P03_result'), ('P04', '_J2_P04_result'), ('P05', '_J2_P05_result'), ('P06', '_J2_P06_result'), ('P07', '_J2_P07_result'), (
    'P08', '_J2_P08_result'), ('P09', '_J2_P09_result'), ('P10', '_J2_P10_result'), ('P11', '_J2_P11_result'), ('P12', '_J2_P12_result'), ('P13', '_J2_P13_result'), ('P14', '_J2_P14_result'), ('P15', '_J2_P15_result'), ('P16', '_J2_P16_result')]


def _filter_cases(cases: Sequence[Tuple[str, str]], keep: Optional[Sequence[str]]) -> List[Tuple[str, str]]:
    if not keep:
        return list(cases)
    keep_set = {k.strip() for k in keep}
    return [c for c in cases if c[0] in keep_set]


def main(
    scheme: str = "both",
    participants: Optional[Sequence[str]] = None,
    base_dir: Optional[Union[str, Path]] = None,
) -> None:
    scheme = scheme.upper()
    if scheme not in {"J1", "J2", "BOTH"}:
        raise ValueError("scheme must be one of: J1, J2, both")

    if scheme in {"J1", "BOTH"}:
        for pxx, path_name in _filter_cases(CASES_J1, participants):
            get_fiber_force_files(pxx, path_name, base_dir=base_dir)

    if scheme in {"J2", "BOTH"}:
        for pxx, path_name in _filter_cases(CASES_J2, participants):
            get_fiber_force_files(pxx, path_name, base_dir=base_dir)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--scheme", default="both", help="J1, J2, or both")
    ap.add_argument("--participants", nargs="*", default=None)
    ap.add_argument("--base_dir", default=None)
    args = ap.parse_args()

    main(scheme=args.scheme, participants=args.participants, base_dir=args.base_dir)
