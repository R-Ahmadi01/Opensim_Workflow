# Opensim_Workflow

# OpenSim MocoInverse Cycling (J1 / J2) + Muscle Fiber Forces + Knee Joint Reaction

This repo contains Python scripts to run **OpenSim MocoInverse** on cycling trials (two schemes: **J1** and **J2**), then export useful outputs like **knee joint reaction forces (JRF)** and **muscle fiber-force / multipliers**.

---

## What’s included

### Main workflows
- **Run MocoInverse for multiple participants (J1, J2, or both)** :contentReference[oaicite:0]{index=0}  
- **Run fiber-force/multiplier exports for multiple participants (J1, J2, or both)** :contentReference[oaicite:1]{index=1}  

### Single-participant solvers
- **J1 solver** (excitation-effort goal weight only) :contentReference[oaicite:2]{index=2}  
- **J2 solver** (adds a knee joint reaction goal on `walker_knee_r`) :contentReference[oaicite:3]{index=3}  

### Post-processing utilities
- **Export muscle fiber forces + multipliers from a MocoInverse solution** :contentReference[oaicite:4]{index=4}  
- **Convert comma-separated external loads → OpenSim-ready `.sto`, patch model XML, and compute knee JRF** :contentReference[oaicite:5]{index=5}  

Common constants/helpers (muscle list, tendon flags, coordinate actuators, CSV utilities, etc.) live here: :contentReference[oaicite:6]{index=6}

---

## Requirements

- Python 3.x
- **OpenSim 4.x Python bindings with Moco enabled** (must be importable as `import opensim as osim`) :contentReference[oaicite:7]{index=7}

---

## Expected input files (per participant)

These scripts assume you have a folder (one per participant or one folder containing all participants) containing files named like:

- `PXX_CyclingModel_rra.osim`
- `PXXexternal_loads_pedal.xml`
- `PXXrra_cycling_Kinematics_q.sto`
- `PXXrra_cycling_states.sto`
- (and a “temp” guess solution will be read/written automatically)

See the solver scripts for the exact filenames they load and write. 

---

## Quick start

### 1) Run MocoInverse for everyone (J1 then J2)
```bash
python run_moco_inverse_cycling.py --scheme both
