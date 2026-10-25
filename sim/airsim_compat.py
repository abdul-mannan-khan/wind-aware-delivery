"""Import the AirSim client from its own folder.

The AirSim client needs tornado 4, Jupyter needs tornado 6. In the Docker image the client and its old
dependencies are installed in /opt/airsim_deps (outside the normal Python path), so both work.
Use `from airsim_compat import airsim` instead of `import airsim`."""
import os
import sys

_DEPS = os.environ.get("AIRSIM_DEPS", "/opt/airsim_deps")
if os.path.isdir(_DEPS) and _DEPS not in sys.path:
    sys.path.insert(0, _DEPS)

import airsim  # noqa: E402,F401
