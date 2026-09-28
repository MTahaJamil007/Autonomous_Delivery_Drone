#!/usr/bin/env bash
# verify_env.sh - does this machine match the one the project was verified on?
#
#   scripts/verify_env.sh
#
# Run it on a new machine after docs/MIGRATION.md, and before `make check`.
# Every line is OK, WARN or FAIL. Exit status is 0 only when nothing FAILs.
#
#   OK    matches the reference machine exactly
#   WARN  differs, but only in a way expected to be harmless - e.g. the OSRF
#         repo shipping a newer Gazebo 8.x patch release than the reference
#         had. Read it and decide; it does not fail the run.
#   FAIL  differs in a way this project is known to be sensitive to
#
# READ-ONLY. It installs nothing and writes nothing: no apt, no pip, no git
# commands that touch an index, and Python runs with bytecode writing off.
#
# WHY THIS EXISTS
# ---------------
# `make check` proves the code is right for the environment it runs in; it
# cannot say whether that environment is the one the code was proved against.
# The traps on a fresh machine are all silent: Humble's default ros-gz bridge
# targets the wrong Gazebo, protobuf 4 makes gz.msgs10 fail to import, and a
# test dependency that was only ever present by accident passes here and not
# there. Each has a line below.
#
# Reference values are from the development laptop on 28 Sep 2026.

set -uo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCK_FILE="$PROJECT_ROOT/constraints/py310-lock.txt"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

REF_UBUNTU="22.04.5 LTS"
REF_PYTHON="3.10.12"
REF_PIP="22.0.2"
REF_ROS_DESKTOP="0.10.0"
REF_GZ_SIM="8.11.0"
REF_ROS_GZ="0.244.12"
REF_GZ_TRANSPORT="13.5.0"
REF_GZ_MSGS="10.3.2"
REF_PX4_COMMIT="4a48525e4505d74970388476c24cc0cc642bdc5a"

n_ok=0
n_warn=0
n_fail=0

report() { # report STATUS NAME DETAIL
    printf '  %-5s %-26s %s\n' "$1" "$2" "$3"
    case "$1" in
        OK) n_ok=$((n_ok + 1)) ;;
        WARN) n_warn=$((n_warn + 1)) ;;
        *) n_fail=$((n_fail + 1)) ;;
    esac
}

# "install ok installed|<version>", or empty when the package is unknown.
deb_state() { dpkg-query -W -f='${Status}|${Version}' "$1" 2>/dev/null; }

# Installed, and its upstream version (epoch and Debian revision stripped)
# equals the reference: OK. Installed at another version: WARN. Absent: FAIL.
check_deb() { # check_deb PACKAGE REFERENCE_VERSION
    local state version
    state="$(deb_state "$1")"
    if [[ "$state" != "install ok installed|"* ]]; then
        report FAIL "$1" "not installed"
        return
    fi
    version="${state#*|}"
    version="${version#*:}"
    version="${version%%-*}"
    if [ "$version" = "$2" ]; then
        report OK "$1" "$version"
    else
        report WARN "$1" "$version (reference: $2)"
    fi
}

echo "System"
if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    ubuntu_version="$(. /etc/os-release && echo "${VERSION_ID:-}|${VERSION:-}")"
    if [ "${ubuntu_version%%|*}" != "22.04" ]; then
        report FAIL "Ubuntu" "${ubuntu_version#*|} - ROS 2 Humble and this lock file need 22.04"
    elif [ "${ubuntu_version#*|}" = "$REF_UBUNTU (Jammy Jellyfish)" ]; then
        report OK "Ubuntu" "$REF_UBUNTU"
    else
        report WARN "Ubuntu" "${ubuntu_version#*|} (reference: $REF_UBUNTU)"
    fi
else
    report FAIL "Ubuntu" "/etc/os-release not readable"
fi

python_path="$(command -v python3 || true)"
if [ -z "$python_path" ]; then
    report FAIL "python3" "not on PATH"
else
    python_version="$(python3 -c 'import platform; print(platform.python_version())')"
    if [ "$python_version" = "$REF_PYTHON" ] && [ "$python_path" = /usr/bin/python3 ]; then
        report OK "python3" "$python_version ($python_path)"
    elif [[ "$python_version" != 3.10.* ]]; then
        report FAIL "python3" "$python_version ($python_path) - the lock file is for CPython 3.10; is a venv or conda active?"
    else
        report WARN "python3" "$python_version ($python_path) (reference: $REF_PYTHON, /usr/bin/python3)"
    fi
    pip_version="$(python3 -m pip --version 2>/dev/null | awk '{print $2}')"
    if [ "$pip_version" = "$REF_PIP" ]; then
        report OK "pip" "$pip_version"
    elif [ -z "$pip_version" ]; then
        report FAIL "pip" "not installed - sudo apt install python3-pip"
    else
        report WARN "pip" "$pip_version (reference: $REF_PIP)"
    fi
fi

echo
echo "ROS 2 and Gazebo"
check_deb ros-humble-desktop "$REF_ROS_DESKTOP"
if [ "${ROS_DISTRO:-}" = "humble" ]; then
    report OK "ROS environment" "sourced (ROS_DISTRO=humble)"
else
    report FAIL "ROS environment" "not sourced - add 'source /opt/ros/humble/setup.bash' to ~/.bashrc and open a new terminal"
fi
check_deb ros-humble-ros-gzharmonic "$REF_ROS_GZ"

# Humble's default ros-gz bridge is built for Gazebo Fortress. Its packages
# declare a conflict with the Harmonic ones, and a bridge for the wrong
# Gazebo leaves the drone with no LiDAR feed - it then holds position forever.
fortress_bridge="$(dpkg-query -W -f='${Package}|${Status}\n' 'ros-humble-ros-gz*' 2>/dev/null \
    | awk -F'|' '$2 == "install ok installed" && $1 !~ /^ros-humble-ros-gzharmonic/ {print $1}' \
    | tr '\n' ' ')"
if [ -z "$fortress_bridge" ]; then
    report OK "no Fortress ros-gz bridge" "ros-humble-ros-gz* absent"
else
    report FAIL "Fortress ros-gz bridge" "installed: ${fortress_bridge% } - remove it; it targets the wrong Gazebo"
fi

if command -v gz >/dev/null 2>&1; then
    gz_sim_version="$(timeout 30 gz sim --versions 2>/dev/null | head -n 1)"
    if [ "$gz_sim_version" = "$REF_GZ_SIM" ]; then
        report OK "gz sim" "$gz_sim_version"
    elif [[ "$gz_sim_version" == 8.* ]]; then
        report WARN "gz sim" "$gz_sim_version (reference: $REF_GZ_SIM)"
    else
        report FAIL "gz sim" "'${gz_sim_version:-no output}' - need Gazebo Harmonic (gz-sim 8)"
    fi
else
    report FAIL "gz sim" "gz not on PATH - Gazebo Harmonic is not installed"
fi
check_deb python3-gz-transport13 "$REF_GZ_TRANSPORT"
check_deb python3-gz-msgs10 "$REF_GZ_MSGS"

echo
echo "PX4 ($PX4_DIR)"
if [ -d "$PX4_DIR/.git" ]; then
    px4_commit="$(git -C "$PX4_DIR" rev-parse HEAD 2>/dev/null)"
    if [ "$px4_commit" = "$REF_PX4_COMMIT" ]; then
        report OK "PX4 commit" "${px4_commit:0:10}"
    else
        report FAIL "PX4 commit" "${px4_commit:-unknown} (need $REF_PX4_COMMIT)"
    fi
    if [ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
        report OK "PX4 SITL build" "build/px4_sitl_default/bin/px4"
    else
        report FAIL "PX4 SITL build" "missing - run 'make px4_sitl' in $PX4_DIR"
    fi
    # sim/env.sh sources this; it only exists after the build.
    if [ -f "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh" ]; then
        report OK "PX4 gz_env.sh" "present"
    else
        report FAIL "PX4 gz_env.sh" "missing - run 'make px4_sitl' in $PX4_DIR"
    fi
else
    report FAIL "PX4-Autopilot" "not a git checkout at $PX4_DIR (set PX4_DIR if it lives elsewhere)"
fi

echo
echo "Python packages"
if [ ! -f "$LOCK_FILE" ]; then
    report FAIL "lock file" "$LOCK_FILE missing"
elif [ -n "$python_path" ]; then
    # From / so that `import config` has to come through the editable install,
    # exactly as it does for the services, not through the working directory.
    # The Python half ends with an END line. Without it, a crash there would
    # print no FAIL at all and this script would report a clean pass.
    python_completed=no
    while IFS='|' read -r status name detail; do
        if [ "$status" = END ]; then
            python_completed=yes
        else
            report "$status" "$name" "$detail"
        fi
    done < <(cd / && PYTHONDONTWRITEBYTECODE=1 python3 - "$PROJECT_ROOT" "$LOCK_FILE" <<'PY'
import importlib
import importlib.metadata as metadata
import os
import sys
from pathlib import Path

project_root, lock_file = Path(sys.argv[1]), sys.argv[2]


def report(status, name, detail):
    print(f"{status}|{name}|{detail}")


def installed(name):
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return None


def check_pins(label, pins):
    wrong = [(name, installed(name), want) for name, want in pins.items()]
    wrong = [entry for entry in wrong if entry[1] != entry[2]]
    for name, have, want in wrong:
        report("FAIL", name, f"{have or 'not installed'} (expected {want})")
    if not wrong:
        report("OK", label, f"all {len(pins)} match")


pip_pins = {}
with open(lock_file) as lock:
    for raw in lock:
        line = raw.split("#", 1)[0].strip()
        if line:
            name, _, version = line.partition("==")
            pip_pins[name.strip()] = version.strip()
check_pins("pip packages (lock file)", pip_pins)

# Not in the lock file: apt provides these, via ros-humble-desktop, colcon
# and python3-pip, and pip leaves them alone because they already satisfy
# every requirement. A different version here means pip replaced one.
check_pins(
    "apt Python packages",
    {
        "empy": "3.3.4",
        "idna": "3.3",
        "iniconfig": "1.1.1",
        "pillow": "9.0.1",
        "pygments": "2.11.2",
        "pytz": "2022.1",
        "pyyaml": "5.4.1",
        "six": "1.16.0",
        "toml": "0.10.2",
        "urllib3": "1.26.5",
        "wheel": "0.37.1",
    },
)

# opencv-python and opencv-contrib-python unpack into the same cv2/ directory,
# so having both leaves whichever was installed last - possibly without aruco.
if installed("opencv-python"):
    report("FAIL", "opencv-python", "installed alongside opencv-contrib-python; uninstall it")


def check_import(label, check):
    try:
        detail = check()
    except Exception as error:  # the message is the diagnosis
        report("FAIL", label, f"{type(error).__name__}: {error}")
    else:
        report("OK", label, detail)


def gz_bindings():
    importlib.import_module("gz.transport13")
    importlib.import_module("gz.msgs10.image_pb2")
    return "gz.transport13, gz.msgs10 import"


def opencv_aruco():
    import cv2

    for name in ("ArucoDetector", "detectMarkers", "getPredefinedDictionary"):
        getattr(cv2.aruco, name)
    return f"cv2 {cv2.__version__} with aruco"


def mavsdk_server():
    import mavsdk

    binary = Path(mavsdk.__file__).parent / "bin" / "mavsdk_server"
    if not os.access(binary, os.X_OK):
        raise FileNotFoundError(f"{binary} missing or not executable")
    return "mavsdk_server binary present"


def ros_python():
    importlib.import_module("rclpy")
    importlib.import_module("sensor_msgs.msg")
    return "rclpy, sensor_msgs import"


def project_install():
    import config

    location = Path(config.__file__).resolve()
    if project_root not in location.parents:
        raise ImportError(f"config resolves to {location}, not this checkout - run make install here")
    return "editable install points at this checkout"


check_import("Gazebo Python bindings", gz_bindings)
check_import("OpenCV", opencv_aruco)
check_import("MAVSDK", mavsdk_server)
check_import("ROS 2 Python", ros_python)
check_import("project", project_install)
print("END||")
PY
    )
    if [ "$python_completed" != yes ]; then
        report FAIL "Python checks" "did not run to completion - see the traceback above"
    fi
fi

echo
echo "verify_env: $n_ok OK, $n_warn WARN, $n_fail FAIL"
if [ "$n_fail" -eq 0 ]; then
    echo "verify_env: environment matches the reference machine"
    exit 0
fi
echo "verify_env: environment does NOT match - fix every FAIL above, then run make check"
exit 1
