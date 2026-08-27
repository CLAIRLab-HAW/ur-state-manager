"""Put ``src/ur_state_manager/`` on ``sys.path`` so the tests can import the package without installing it.

This package is built by colcon (``ament_python``), not by ``uv``: it has no ``pyproject.toml`` and no entry in
``[tool.uv.workspace].members`` -- deliberately, see the "seven remaining repos" paragraph in CLAUDE.md, where a
``pyproject.toml`` here would make ``uv`` take the sub-repo for the project root.  So nothing installs
``ur_state_manager`` into the workspace venv, and an import would fail even though the root run collects these
files by path (the same arrangement as ``deploy/husky-offboard/tests``, which is likewise not a member).

Only the ROS-free modules are importable this way.  ``state_manager`` and ``controller_mode_manager`` pull in
``rclpy`` and ``ur_dashboard_msgs``, which exist in neither the Mac venv nor the offboard images -- that is exactly
why the decisions worth testing live in ``readiness`` and ``switching`` instead.
"""

from __future__ import annotations

import sys
from pathlib import Path

_SRC = str(Path(__file__).resolve().parent.parent / "src" / "ur_state_manager")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
