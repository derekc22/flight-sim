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

python3 "$SCRIPT"   -d "$DATA_DIR/p.csv"              -l "x(t) [m]"           "y(t) [m]"          "z(t) [m]"                                              -t "position vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/eul.csv"            -l "psi(t) [rad]"       "theta(t) [rad]"    "phi(t) [rad]"                                          -t "orientation vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/w.csv"              -l "p(t) [rad/s]"       "q(t) [rad/s]"      "r(t) [rad/s]"                                          -t "angular velocity vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/v.csv"              -l "vx(t) [m/s]"        "vy(t) [m/s]"       "vz(t) [m/s]"                                           -t "linear velocity vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/u_surface.csv"      -l "elevator(t) [rad]"  "aileron(t) [rad]"  "rudder(t) [rad]"  "flap(t) [rad]"  "spoiler(t) [rad]"  -t "surface inputs vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/u_propulsor.csv"    -l "front(t) [N]"       "left(t) [N]"       "right(t) [N]"                                          -t "propulsor inputs vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/F_net.csv"          -l "Fx_net(t) [N]"      "Fy_net(t) [N]"     "Fz_net(t) [N]"                                         -t "net force vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/M_net.csv"          -l "Mx_net(t) [Nm]"     "My_net(t) [Nm]"    "Mz_net(t) [Nm]"                                        -t "net moment vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/F_aero.csv"         -l "Fx_aero(t) [N]"     "Fy_aero(t) [N]"    "Fz_aero(t) [N]"                                        -t "aerodynamic force vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/M_aero.csv"         -l "Mx_aero(t) [Nm]"    "My_aero(t) [Nm]"   "Mz_aero(t) [Nm]"                                       -t "aerodynamic moment vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/F_prop.csv"         -l "Fx_prop(t) [N]"     "Fy_prop(t) [N]"    "Fz_prop(t) [N]"                                        -t "propulsive force vs time"
python3 "$SCRIPT"   -d "$DATA_DIR/M_prop.csv"         -l "Mx_prop(t) [Nm]"    "My_prop(t) [Nm]"   "Mz_prop(t) [Nm]"                                       -t "propulsive moment vs time"