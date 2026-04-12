#!/bin/bash
set -e

CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate ame532

SCRIPT="scripts/subplot.py"

if [ -z "$1" ]; then
  echo "Usage: $0 <DATA_DIR>"
  exit 1
fi

DATA_DIR="$1"

python3 "$SCRIPT"   -d "$DATA_DIR/p.csv"    -l "x(t) [m]"           "y(t) [m]"          "z(t) [m]"                                              -t "position vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/eul.csv"  -l "psi(t) [rad]"       "theta(t) [rad]"    "phi(t) [rad]"                                          -t "orientation vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/w.csv"    -l "p(t) [rad/s]"       "q(t) [rad/s]"      "r(t) [rad/s]"                                          -t "angular velocity vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/v.csv"    -l "vx(t) [m/s]"        "vy(t) [m/s]"       "vz(t) [m/s]"                                           -t "linear velocity vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/u.csv"    -l "elevator(t) [rad]"  "aileron(t) [rad]"  "rudder(t) [rad]"  "flaps(t) [rad]"  "spoilers(t) [rad]"  -t "control inputs vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/F.csv"    -l "Fx(t) [N]"          "Fy(t) [N]"         "Fz(t) [N]"                                             -t "net force vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/M.csv"    -l "Mx(t) [Nm]"         "My(t) [Nm]"        "Mz(t) [Nm]"                                            -t "net moment vs time"