from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import opensim as osim


def parse_external_loads_xml(xml_path: Path) -> tuple[str, list[str]]:
    """Return (datafile_name, required_column_names) from ExternalLoads XML."""
    tree = ET.parse(xml_path)
    root = tree.getroot()

    ext = root.find("ExternalLoads")
    if ext is None:
        raise RuntimeError("No <ExternalLoads> element found in the XML.")

    datafile = (ext.findtext("datafile") or "").strip()
    if not datafile:
        raise RuntimeError(
            "No <datafile> found (or it is empty) in ExternalLoads XML.")

    prefixes: list[str] = []
    objs = ext.find("objects")
    if objs is None:
        raise RuntimeError("No <objects> found under <ExternalLoads>.")

    for ef in objs.findall("ExternalForce"):
        for tag in ("force_identifier", "point_identifier", "torque_identifier"):
            v = (ef.findtext(tag) or "").strip()
            if v:
                prefixes.append(v)

    required = ["time"]
    for p in prefixes:
        for ax in "xyz":
            # NOTE: torque_identifier has trailing '_' -> right_torque_x etc.
            required.append(f"{p}{ax}")

    # unique preserve order
    seen = set()
    required_unique = []
    for c in required:
        if c not in seen:
            required_unique.append(c)
            seen.add(c)

    return datafile, required_unique


def read_header_and_labels(storage_path: Path) -> tuple[list[str], list[str], str, int | None, int | None]:
    """
    Read until 'endheader', then read label line.
    Detect delimiter (comma vs tab) for the label line.
    """
    header_lines: list[str] = []
    labels_line = None

    with storage_path.open("r", encoding="utf-8-sig", errors="replace") as f:
        for line in f:
            header_lines.append(line.rstrip("\r\n"))
            if line.strip().lower() == "endheader":
                labels_line = next(f).rstrip("\r\n")
                break

    if labels_line is None:
        raise RuntimeError(
            f"Could not find 'endheader' in {storage_path.name}")

    # delimiter detect (file is comma-separated after endheader)
    if "," in labels_line and "\t" not in labels_line:
        delim = ","
    elif "\t" in labels_line and "," not in labels_line:
        delim = "\t"
    else:
        try:
            dialect = csv.Sniffer().sniff(
                labels_line, delimiters=[",", "\t", ";"])
            delim = dialect.delimiter
        except Exception:
            delim = ","

    labels = [x.strip() for x in labels_line.split(delim) if x.strip() != ""]

    datacolumns = None
    datarows = None
    for hl in header_lines:
        s = hl.strip().lower()
        if s.startswith("datacolumns"):
            parts = s.split()
            if len(parts) >= 2:
                datacolumns = int(parts[1])
        if s.startswith("datarows"):
            parts = s.split()
            if len(parts) >= 2:
                datarows = int(parts[1])

    return header_lines, labels, delim, datacolumns, datarows


def convert_storage_commas_to_sto(input_path: Path, output_path: Path, required_cols: list[str]) -> None:
    """
    Convert a Storage-like file whose labels+data are comma-separated into a tab-delimited .sto.
    """
    _, labels, delim, datacol, _ = read_header_and_labels(input_path)

    if datacol is not None and datacol != len(labels):
        # Not fatal, but warn via exception for clarity (In case of using this code it should match: 19)
        raise RuntimeError(
            f"Header says datacolumns={datacol} but parsed {len(labels)} labels from label line."
        )

    missing = [c for c in required_cols if c not in labels]
    if missing:
        raise RuntimeError(
            "Pedal force file is missing required columns from ExternalLoads XML.\n"
            f"Missing ({len(missing)}): {missing}\n"
            f"Found labels ({len(labels)}): {labels}"
        )

    n_cols = len(labels)

    # Count rows (cheap enough for ~80k lines)
    n_rows = 0
    with input_path.open("r", encoding="utf-8-sig", errors="replace") as f:
        for line in f:
            if line.strip().lower() == "endheader":
                break
        next(f)  # labels line
        for line in f:
            if line.strip():
                n_rows += 1

    # Write OpenSim .sto (tab-delimited)
    with output_path.open("w", encoding="utf-8", newline="") as out:
        out.write(f"{output_path.stem}\n")
        out.write("version=1\n")
        out.write(f"nRows={n_rows}\n")
        out.write(f"nColumns={n_cols}\n")
        out.write("inDegrees=no\n")
        out.write("endheader\n")
        out.write("\t".join(labels) + "\n")

        with input_path.open("r", encoding="utf-8-sig", errors="replace") as f:
            for line in f:
                if line.strip().lower() == "endheader":
                    break
            next(f)  # labels line

            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = [x.strip() for x in line.split(delim)]
                if len(parts) < n_cols:
                    parts += [""] * (n_cols - len(parts))
                elif len(parts) > n_cols:
                    parts = parts[:n_cols]
                out.write("\t".join(parts) + "\n")


def patch_model_external_loads(model_in: Path, model_out: Path, old_name: str, new_name: str) -> int:
    """
    Patch <datafile> and <data_source_name> inside the model .osim if they match old_name.
    """
    tree = ET.parse(model_in)
    root = tree.getroot()
    patched = 0

    for el in root.iter():
        tag = el.tag.split("}")[-1]
        if tag in ("datafile", "data_source_name"):
            if (el.text or "").strip() == old_name:
                el.text = new_name
                patched += 1

    tree.write(model_out, encoding="UTF-8", xml_declaration=True)
    return patched


def compute_knee_jrf(model_path: Path, solution_path: Path, out_csv: Path, base_dir: Path) -> None:
    """
    Compute knee joint reaction (reaction on child expressed in ground) for walker_knee_r and walker_knee_l.
    """
    old_cwd = os.getcwd()
    os.chdir(str(base_dir))  # so relative datafile resolves correctly
    try:
        model = osim.Model(str(model_path))
        model.initSystem()

        traj = osim.MocoTrajectory(str(solution_path))
        states_table = traj.exportToStatesTable()
        states_traj = osim.StatesTrajectory.createFromStatesTable(
            model, states_table)

        knee_r = model.getJointSet().get("walker_knee_r")
        knee_l = model.getJointSet().get("walker_knee_l")

        with out_csv.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "time",
                "knee_r_Mx", "knee_r_My", "knee_r_Mz", "knee_r_Fx", "knee_r_Fy", "knee_r_Fz",
                "knee_l_Mx", "knee_l_My", "knee_l_Mz", "knee_l_Fx", "knee_l_Fy", "knee_l_Fz",
            ])

            n = states_traj.getSize() if hasattr(states_traj, "getSize") else len(states_traj)
            for i in range(n):
                s = states_traj[i]
                model.realizeAcceleration(s)

                rr = knee_r.calcReactionOnChildExpressedInGround(s)
                rl = knee_l.calcReactionOnChildExpressedInGround(s)

                Mr, Fr = rr.get(0), rr.get(1)
                Ml, Fl = rl.get(0), rl.get(1)

                w.writerow([
                    float(s.getTime()),
                    float(Mr.get(0)), float(Mr.get(1)), float(Mr.get(2)),
                    float(Fr.get(0)), float(Fr.get(1)), float(Fr.get(2)),
                    float(Ml.get(0)), float(Ml.get(1)), float(Ml.get(2)),
                    float(Fl.get(0)), float(Fl.get(1)), float(Fl.get(2)),
                ])
    finally:
        os.chdir(old_cwd)


def main(pxx: str, suffix: str, base_dir: Path, loads_xml: Path) -> None:
    base_dir = Path(base_dir)

    model_in = base_dir / f"{pxx}_CyclingModel_moco{suffix}.osim"
    solution = base_dir / f"{pxx}Cycling_MocoInverse_solution{suffix}.sto"

    if not model_in.exists():
        raise FileNotFoundError(f"Model not found: {model_in}")
    if not solution.exists():
        raise FileNotFoundError(f"Solution not found: {solution}")
    if not loads_xml.exists():
        raise FileNotFoundError(f"ExternalLoads XML not found: {loads_xml}")

    datafile_name, required_cols = parse_external_loads_xml(loads_xml)

    datafile_path = base_dir / datafile_name
    if not datafile_path.exists():
        raise FileNotFoundError(
            f"External loads data file not found: {datafile_path}")

    # Convert to OpenSim-friendly .sto (tab-delimited)
    sto_name = Path(datafile_name).with_suffix("").name + "_opensim.sto"
    sto_path = base_dir / sto_name
    convert_storage_commas_to_sto(datafile_path, sto_path, required_cols)
    print(f"[ok] wrote OpenSim-ready loads file: {sto_path.name}")

    # Patch model to point to the new .sto
    model_out = base_dir / f"{pxx}_CyclingModel_moco{suffix}_extfixed.osim"
    patched = patch_model_external_loads(
        model_in, model_out, datafile_name, sto_name)
    print(
        f"[ok] wrote patched model: {model_out.name} (patched {patched} tags)")

    # Compute JRF
    out_csv = base_dir / f"{pxx}{suffix}_JRF_full.csv"
    compute_knee_jrf(model_out, solution, out_csv, base_dir)
    print(f"[ok] wrote JRF: {out_csv.name}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("pxx", help="Participant, e.g., P01")
    ap.add_argument("suffix", help="Suffix, e.g., _J1_P01_result")
    ap.add_argument("base_dir", nargs="?", default=".",
                    help="Folder containing model/solution/loads (default: .)")
    ap.add_argument("--loads_xml", required=True,
                    help="Path to external loads XML (e.g., P01external_loads_pedal.xml)")
    args = ap.parse_args()

    main(args.pxx, args.suffix, Path(args.base_dir), Path(args.loads_xml))
