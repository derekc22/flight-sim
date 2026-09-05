#!/bin/bash
set -e

source .env

CONDA_BASE_PATH=$(conda info --base)
source "$CONDA_BASE_PATH/etc/profile.d/conda.sh"
conda activate ame532

PLOT_2D_PATH="$PROJ_PATH/plot/plot_2d.py"
PLOT_3D_PATH="$PROJ_PATH/plot/plot_3d.py"

if [ -z "$1" ]; then
	echo "Usage: $0 <OUT_DIR>"
	exit 1
fi

OUT_DIR="$1"
DATA_DIR_PATH="$PROJ_PATH/results/$OUT_DIR/data"
PLOT_DIR_PATH="$PROJ_PATH/results/$OUT_DIR/figures/run"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/p.csv" -s "$PLOT_DIR_PATH" \
-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
-t "position"

if [ -f "$DATA_DIR_PATH/p_measured.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/p_measured.csv" -r "$DATA_DIR_PATH/p.csv" -s "$PLOT_DIR_PATH" \
	-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
	-t "position measurement vs position"
fi

if [ -f "$DATA_DIR_PATH/p_estimated.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/p_estimated.csv" -r "$DATA_DIR_PATH/p.csv" -s "$PLOT_DIR_PATH" \
	-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
	-t "position estimate vs position"
fi

if [ -f "$DATA_DIR_PATH/eul_setpoint.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/eul.csv" -r "$DATA_DIR_PATH/eul_setpoint.csv" -s "$PLOT_DIR_PATH" \
	-l "psi(t) [rad]" "theta(t) [rad]" "phi(t) [rad]" \
	-t "orientation vs setpoint"
else
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/eul.csv" -s "$PLOT_DIR_PATH" \
	-l "psi(t) [rad]" "theta(t) [rad]" "phi(t) [rad]" \
	-t "orientation"
fi

if [ -f "$DATA_DIR_PATH/eul_measured.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/eul_measured.csv" -r "$DATA_DIR_PATH/eul.csv" -s "$PLOT_DIR_PATH" \
	-l "psi(t) [rad]" "theta(t) [rad]" "phi(t) [rad]" \
	-t "orientation measurement vs orientation"
fi

if [ -f "$DATA_DIR_PATH/eul_estimated.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/eul_estimated.csv" -r "$DATA_DIR_PATH/eul.csv" -s "$PLOT_DIR_PATH" \
	-l "psi(t) [rad]" "theta(t) [rad]" "phi(t) [rad]" \
	-t "orientation estimate vs orientation"
fi

if [ -f "$DATA_DIR_PATH/w_setpoint.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/w.csv" -r "$DATA_DIR_PATH/w_setpoint.csv" -s "$PLOT_DIR_PATH" \
	-l "p(t) [rad/s]" "q(t) [rad/s]" "r(t) [rad/s]" \
	-t "angular velocity vs setpoint"
else
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/w.csv" -s "$PLOT_DIR_PATH" \
	-l "p(t) [rad/s]" "q(t) [rad/s]" "r(t) [rad/s]" \
	-t "angular velocity"
fi

if [ -f "$DATA_DIR_PATH/w_measured.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/w_measured.csv" -r "$DATA_DIR_PATH/w.csv" -s "$PLOT_DIR_PATH" \
	-l "p(t) [rad/s]" "q(t) [rad/s]" "r(t) [rad/s]" \
	-t "angular velocity measurement vs angular velocity"
fi

if [ -f "$DATA_DIR_PATH/w_estimated.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/w_estimated.csv" -r "$DATA_DIR_PATH/w.csv" -s "$PLOT_DIR_PATH" \
	-l "p(t) [rad/s]" "q(t) [rad/s]" "r(t) [rad/s]" \
	-t "angular velocity estimate vs angular velocity"
fi

if [ -f "$DATA_DIR_PATH/v_setpoint.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/v.csv" -r "$DATA_DIR_PATH/v_setpoint.csv" -s "$PLOT_DIR_PATH" \
	-l "vx(t) [m/s]" "vy(t) [m/s]" "vz(t) [m/s]" \
	-t "linear velocity vs setpoint"
else
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/v.csv" -s "$PLOT_DIR_PATH" \
	-l "vx(t) [m/s]" "vy(t) [m/s]" "vz(t) [m/s]" \
	-t "linear velocity"
fi

if [ -f "$DATA_DIR_PATH/v_measured.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/v_measured.csv" -r "$DATA_DIR_PATH/v.csv" -s "$PLOT_DIR_PATH" \
	-l "vx(t) [m/s]" "vy(t) [m/s]" "vz(t) [m/s]" \
	-t "linear velocity measurement vs linear velocity"
fi

if [ -f "$DATA_DIR_PATH/v_estimated.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/v_estimated.csv" -r "$DATA_DIR_PATH/v.csv" -s "$PLOT_DIR_PATH" \
	-l "vx(t) [m/s]" "vy(t) [m/s]" "vz(t) [m/s]" \
	-t "linear velocity estimate vs linear velocity"
fi

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/u_surface.csv" -r "$DATA_DIR_PATH/u_surface_commanded.csv" -s "$PLOT_DIR_PATH" \
-l "elevator(t) [rad]" "aileron(t) [rad]" "rudder(t) [rad]" "flap(t) [rad]" "spoiler(t) [rad]" \
-t "surface inputs vs commanded surface inputs"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/u_propulsor.csv" -r "$DATA_DIR_PATH/u_propulsor_commanded.csv" -s "$PLOT_DIR_PATH" \
-l "front(t) [N]" "left(t) [N]" "right(t) [N]" \
-t "propulsor inputs vs commanded propulsor inputs"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/F_net.csv" -s "$PLOT_DIR_PATH" \
-l "Fx(t) [N]" "Fy(t) [N]" "Fz(t) [N]" \
-t "net force"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/M_net.csv" -s "$PLOT_DIR_PATH" \
-l "Mx(t) [Nm]" "My(t) [Nm]" "Mz(t) [Nm]" \
-t "net moment"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/F_aerodynamic.csv" -s "$PLOT_DIR_PATH" \
-l "Fx(t) [N]" "Fy(t) [N]" "Fz(t) [N]" \
-t "aerodynamic force"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/M_aerodynamic.csv" -s "$PLOT_DIR_PATH" \
-l "Mx(t) [Nm]" "My(t) [Nm]" "Mz(t) [Nm]" \
-t "aerodynamic moment"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/F_propulsive.csv" -s "$PLOT_DIR_PATH" \
-l "Fx(t) [N]" "Fy(t) [N]" "Fz(t) [N]" \
-t "propulsive force"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/M_propulsive.csv" -s "$PLOT_DIR_PATH" \
-l "Mx(t) [Nm]" "My(t) [Nm]" "Mz(t) [Nm]" \
-t "propulsive moment"

if [ -f "$DATA_DIR_PATH/windB.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/windB.csv" -s "$PLOT_DIR_PATH" \
	-l "windB_x(t) [m/s]" "windB_y(t) [m/s]" "windB_z(t) [m/s]" \
	-t "wind"
fi

python "$PLOT_3D_PATH" -d "$DATA_DIR_PATH/p.csv" -s "$PLOT_DIR_PATH" \
-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
-t "position"
