# Scanning Measurement Path Planning Software

A research-oriented software framework for robotic scanning measurement path planning based on intelligent optimization algorithms.

The current integrated branch adds constrained TCP speed planning,
scanner-to-tool calibration handling, and RoboDK program generation. See the
[documentation index](docs/README.md) for the runnable workflow, architecture,
validation boundaries, and research plans.

> **Scope note:** the software currently orders discrete path points and plans
> speed along that path. Geometric path smoothing is documented as future work;
> it is not yet implemented.

## Overview

This repository focuses on robotic scanning measurement and path planning for free-form surface inspection scenarios. The project integrates viewpoint planning, path optimization, and intelligent evolutionary algorithms to improve scanning efficiency, trajectory smoothness, and measurement stability.

The software is developed primarily for:

- Robotic scanning measurement
- Free-form surface inspection
- Multi-viewpoint path planning
- Intelligent optimization algorithms
- Genetic algorithm based trajectory optimization
- Robot motion sequence optimization
- CAD model based scanning trajectory planning

## Research Background

In robotic scanning measurement systems, the quality of the scanning trajectory directly influences:

- Measurement efficiency
- Robot motion smoothness
- Scanner stability
- Surface coverage completeness
- Overall inspection accuracy

Traditional shortest-path optimization methods mainly focus on geometric distance minimization, while often neglecting pose variation and robot motion fluctuation between adjacent viewpoints. This project aims to address these limitations through collaborative evolutionary optimization strategies.

## Main Features

- Multi-strategy collaborative genetic algorithm (MSCGA)
- Robotic scanning path optimization
- Pose-constrained viewpoint sequencing
- Path length and pose variation collaborative optimization
- Evolutionary operator coordination mechanism
- CAD-based geometric processing
- PythonOCC geometric modeling and visualization
- PyQt5 graphical user interface
- STEP/IGES drag-and-drop model import
- JSON-driven English/Simplified Chinese UI switching
- Pose-only planned-path import to RoboDK before speed planning
- Research-oriented experimental framework

## Repository Structure

```text
Scanning-measurement-path-planning-software/
│
├── main.py                    # Desktop workflow and application entry
├── geometry.py                # CAD geometry and segmentation
├── viewpoints.py              # Viewpoint and pose generation
├── planning.py                # Path-ordering algorithms
├── speed_planning_core.py     # Constrained speed planning
├── pose_transform.py          # Scanner/tool frame conversion
├── robodk_bridge.py           # RoboDK program generation
├── calibration/               # Versioned calibration example
├── docs/                      # User, architecture, plan, and history docs
├── diagnostics/               # Local-only historical diagnostics
├── archive/                   # Local-only tool settings
├── tests/                     # Automated validation
├── run_integrated_app.ps1     # Existing-environment launcher
└── README.md
```

## Methodology

The proposed framework combines:

1. CAD model processing
2. Multi-viewpoint representation
3. Path sequence optimization
4. Pose variation evaluation
5. Multi-strategy collaborative evolution
6. Adaptive operator coordination

The optimization objective jointly considers:

- Geometric path length
- Orientation variation between viewpoints
- Robot motion stability
- Overall scanning efficiency

## Development Environment

### Core Environment

- Python 3.x
- PythonOCC
- PyQt5

### Scientific Computing Libraries

- NumPy
- SciPy
- Matplotlib

### Functional Modules

- CAD geometric processing
- Free-form surface visualization
- Scanning viewpoint interaction
- Evolutionary optimization
- Robot trajectory rendering

## Potential Application Scenarios

- Industrial robotic inspection
- Free-form surface scanning
- Aerospace component inspection
- Precision measurement systems
- Intelligent manufacturing
- Digital twin based inspection planning

## Current Status

This repository is currently under active research and development.

Future updates may include:

- Additional optimization strategies
- Comparative experiments
- Advanced visualization modules
- Robot kinematic constraints
- Hardware integration interfaces
- Complete experimental datasets
- Interactive scanning planning GUI

## Citation

If this repository contributes to your research, please cite the corresponding publication after formal publication.

## License

Currently not specified.

## Author

Jianeng Xiao
