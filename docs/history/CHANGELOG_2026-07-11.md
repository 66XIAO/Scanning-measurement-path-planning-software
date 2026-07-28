# Integration changelog - 2026-07-11

- Added constrained deterministic and Double-Q speed planners.
- Added Pose+Speed CSV export with explicit units and diagnostics.
- Added RoboDK `Set Speed -> Move` bridge, temporary-build validation and
  failure-safe replacement.
- Added integrated Qt speed-planning/RoboDK dialogs and menus.
- Made CAD model import asynchronous and state-transactional.
- Made face segmentation asynchronous.
- Tightened global Close event filtering to the main window only.
- Added PyQt5/PySide6 and Signal compatibility.
- Added no-install Conda bootstrap and launcher.
- Separated TCP angular diagnostics from RoboDK joint command limits.
- Added nine algorithm, CSV interoperability and mocked RoboDK transaction tests.
- Documented thesis mapping, dynamic-validation boundary and geodesic sampling roadmap.
- Added a concise user summary and dependency-ordered next-phase TODO plan.
- Independently verified the Trae review: reproduced a 6.0085% trimmed-surface
  overlap, added per-split/global area-conservation diagnostics and conservative
  fallback, exposed the acceleration margin, and unified background task runners.
- Added phase-A scanner-to-tool transform configuration, rigid-transform tests,
  source/command pose provenance, metadata sidecars, and a validated-calibration
  safety gate before RoboDK connection.
