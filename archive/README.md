# Archive

This directory contains historical, generated, or non-operational material. Nothing here is part of the recommended simulation or real-vehicle startup path.

## Contents

| Path | Purpose |
| --- | --- |
| `legacy/full_soft/` | Old Python vehicle stack that operates without SLAM. |
| `legacy/Lidar/` | Older distributed LiDAR utilities. |
| `legacy/voiture_system_sim_source/` | Historical simulation source snapshot. |
| `artifacts/` | Images and logs kept for reference. |
| `generated/` | Archived builds, caches, and runtime state. |
| `docs_latex_artifacts/` | Generated LaTeX artifacts. |

Do not add `archive/` to `colcon` base paths or `PYTHONPATH`. Recover individual algorithms into an active workspace through small, reviewed, and tested ports.

See [Known Limitations and Legacy Code](../docs/16_known_limitations_and_legacy_parts.md).
