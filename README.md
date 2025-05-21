# MPC-Autonomous-Ship-Navigation

MATLAB simulator for autonomous ship navigation using Model Predictive Control (MPC), COLREG-compliant collision avoidance, and chart-based path planning.<br>
_Created with MATLAB R2024b. Compatible with MATLAB R2024b and later releases._

2025 © Primož Potočnik, University of Ljubljana, 
[Faculty of Mechanical Engineering](https://www.fs.uni-lj.si/en), [Faculty of Maritime Studies and Transport](https://www.fpp.uni-lj.si/en)

---

## Features

- Coastal path planning using GSHHG cartographic data
- Model Predictive Control (MPC) for ship navigation
- COLREG-compliant collision avoidance
- Multi-ship dynamic encounters and collision avoidance
- Video export of simulation results

---

## Folder Structure

```
├── data/ ...... Coastline data, icons, saved maps and planned route
├── results/ ... Simulation video outputs<br>
├── src/ ....... Core MATLAB simulation scripts<br>
├── main.m ..... Entry-point for simulation<br>
├── LICENCE .... MIT License<br>
└── README.md .. Project overview (this file)
```

---
## Getting Started

### Requirements
- MATLAB R2024b or later
- Mapping Toolbox

### Start simulation
1. Clone or download the repository
2. Open MATLAB and navigate to the project directory
3. Run the simulation script:
```matlab
main
```

---
## 📜  License

This project is licensed under the MIT License. See the LICENCE file for details.

---
## Theory and Associated Paper

This project is based on the research paper:

Potočnik, P. (2025). Model Predictive Control for Autonomous Ship Navigation with COLREG Compliance and Chart-Based Path Planning.
_Journal of Marine Science and Engineering._ [DOI link – to be added when available]

Summary
The paper presents a novel control framework for autonomous ship navigation that integrates:

A nonlinear ship model with predictive control for trajectory optimization.

Real-time interpretation of COLREG collision avoidance rules.

Chart-based coastal path planning using publicly available geospatial datasets.

Robust performance in multi-agent scenarios with dynamic obstacle handling.

This simulator implements the theoretical concepts from the paper and provides a testbed for practical validation of the proposed approach.

---
## Citation

If you use this code, please cite the associated paper:<br>
@article{potocnik2025mpc,
  author  = {Primož Potočnik},
  title   = {Model Predictive Control for Autonomous Ship Navigation with COLREG Compliance and Chart-Based Path Planning},
  journal = {Journal of Marine Science and Engineering},
  year    = {2025},
  doi     = {10.XXXX/jmseXXXXX}
}

---
