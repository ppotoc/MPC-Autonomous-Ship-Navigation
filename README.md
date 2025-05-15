# MPC-Autonomous-Ship-Navigation

MATLAB simulator for autonomous ship navigation using Model Predictive Control (MPC), COLREG-compliant collision avoidance, and chart-based path planning.
_Created with MATLAB R2024b. Compatible with MATLAB R2024b and later releases._

2025 © Primož Potočnik, University of Ljubljana, 
[Faculty of Mechanical Engineering](https://www.fs.uni-lj.si/en), [Faculty of Maritime Studies and Transport](https://www.fpp.uni-lj.si/en)

---

## Features

- Model Predictive Control (MPC) for multi-ship navigation
- COLREG-compliant behavior
- Coastal path planning using GSHHG cartographic data
- Multi-ship dynamic encounters and collision avoidance

---

## Folder Structure

`MPC-Autonomous-Ship-Navigation/`<br>
`├── data/ ......` Coastline data, icons, saved maps and planned route<br>
`├── docs/ ......` Published paper<br>
`├── results/ ...` Saved simulation videos<br>
`├── src/ .......` MATLAB simulation scripts<br>
`├── main.m .....` Main simulation script<br>
`├── LICENCE ....` MIT License<br>
`└── README.md ..` This file

---
## Getting Started

### Requirements
- MATLAB R2024b or newer
- Optimization Toolbox
- Mapping Toolbox (for GSHHG)

### Start simulation
1. Copy the repository to your local folder
2. Run the main simulation: main.m

---
##  License

This project is licensed under the MIT License.

---
## Citation

If you use this code, please cite our paper: 
Potočnik, P. (2025). Model Predictive Control for Autonomous Ship Navigation with COLREG Compliance and Chart-Based Path Planning. Journal of Marine Science and Engineering [DOI].

---
