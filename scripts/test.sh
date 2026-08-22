#!/usr/bin/env bash
# Run the test suite in an environment where pytest can actually collect.
#
# WHY THIS WRAPPER EXISTS
# -----------------------
# ROS 2 Humble installs `launch_testing` pytest plugins via setuptools
# entry points. Those plugins are incompatible with pytest 9 and raise
# PluginValidationError during startup -- BEFORE collection -- so any `pytest`
# invoked from a ROS-sourced shell fails without running a single test. Since
# this project is normally developed in exactly such a shell (rclpy is needed
# for avoider_node), plain `pytest tests/` reported nothing at all.
#
# PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 stops the entry-point scan. Everything the
# suite genuinely needs is then loaded back explicitly with -p.
#
# Pass extra pytest arguments through:
#     scripts/test.sh -k avoider -v
#     scripts/test.sh -m sitl            # opt into the flight tests

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

# Explicit plugin list. asyncio is required by pytest.ini's asyncio_mode=auto;
# without it every async test would be silently skipped rather than failing,
# which is the worse outcome.
PLUGINS=(-p asyncio -p no:cacheprovider)

exec python3 -m pytest "${PLUGINS[@]}" "$@"
