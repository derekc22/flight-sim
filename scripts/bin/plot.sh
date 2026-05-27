#!/bin/bash
set -e

source .env

CONDA_BASE_PATH=$(conda info --base)
source "$CONDA_BASE_PATH/etc/profile.d/conda.sh"
conda activate ame532

PLOT_2D_PATH="$PROJ_PATH/scripts/lib/plot/plot_2d.py"
PLOT_3D_PATH="$PROJ_PATH/scripts/lib/plot/plot_3d.py"

if [ -z "$1" ]; then
	echo "Usage: $0 <OUT_DIR>"
	exit 1
fi

OUT_DIR="$1"
DATA_DIR_PATH="$PROJ_PATH/results/data/$OUT_DIR"
PLOT_DIR_PATH="$PROJ_PATH/results/figures/$OUT_DIR"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/p.csv" -s "$PLOT_DIR_PATH" \
-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
-t "position"

if [ -f "$DATA_DIR_PATH/p_meas.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/p_meas.csv" -r "$DATA_DIR_PATH/p.csv" -s "$PLOT_DIR_PATH" \
	-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
	-t "position measurement vs position"
fi

if [ -f "$DATA_DIR_PATH/p_est.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/p_est.csv" -r "$DATA_DIR_PATH/p.csv" -s "$PLOT_DIR_PATH" \
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

if [ -f "$DATA_DIR_PATH/eul_meas.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/eul_meas.csv" -r "$DATA_DIR_PATH/eul.csv" -s "$PLOT_DIR_PATH" \
	-l "psi(t) [rad]" "theta(t) [rad]" "phi(t) [rad]" \
	-t "orientation measurement vs orientation"
fi

if [ -f "$DATA_DIR_PATH/eul_est.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/eul_est.csv" -r "$DATA_DIR_PATH/eul.csv" -s "$PLOT_DIR_PATH" \
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

if [ -f "$DATA_DIR_PATH/w_meas.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/w_meas.csv" -r "$DATA_DIR_PATH/w.csv" -s "$PLOT_DIR_PATH" \
	-l "p(t) [rad/s]" "q(t) [rad/s]" "r(t) [rad/s]" \
	-t "angular velocity measurement vs angular velocity"
fi

if [ -f "$DATA_DIR_PATH/w_est.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/w_est.csv" -r "$DATA_DIR_PATH/w.csv" -s "$PLOT_DIR_PATH" \
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

if [ -f "$DATA_DIR_PATH/v_meas.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/v_meas.csv" -r "$DATA_DIR_PATH/v.csv" -s "$PLOT_DIR_PATH" \
	-l "vx(t) [m/s]" "vy(t) [m/s]" "vz(t) [m/s]" \
	-t "linear velocity measurement vs linear velocity"
fi

if [ -f "$DATA_DIR_PATH/v_est.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/v_est.csv" -r "$DATA_DIR_PATH/v.csv" -s "$PLOT_DIR_PATH" \
	-l "vx(t) [m/s]" "vy(t) [m/s]" "vz(t) [m/s]" \
	-t "linear velocity estimate vs linear velocity"
fi

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/u_surface.csv" -s "$PLOT_DIR_PATH" \
-l "elevator(t) [rad]" "aileron(t) [rad]" "rudder(t) [rad]" "flap(t) [rad]" "spoiler(t) [rad]" \
-t "surface inputs"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/u_propulsor.csv" -s "$PLOT_DIR_PATH" \
-l "front(t) [N]" "left(t) [N]" "right(t) [N]" \
-t "propulsor inputs"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/F_net.csv" -s "$PLOT_DIR_PATH" \
-l "Fx_net(t) [N]" "Fy_net(t) [N]" "Fz_net(t) [N]" \
-t "net force"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/M_net.csv" -s "$PLOT_DIR_PATH" \
-l "Mx_net(t) [Nm]" "My_net(t) [Nm]" "Mz_net(t) [Nm]" \
-t "net moment"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/F_aero.csv" -s "$PLOT_DIR_PATH" \
-l "Fx_aero(t) [N]" "Fy_aero(t) [N]" "Fz_aero(t) [N]" \
-t "aerodynamic force"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/M_aero.csv" -s "$PLOT_DIR_PATH" \
-l "Mx_aero(t) [Nm]" "My_aero(t) [Nm]" "Mz_aero(t) [Nm]" \
-t "aerodynamic moment"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/F_prop.csv" -s "$PLOT_DIR_PATH" \
-l "Fx_prop(t) [N]" "Fy_prop(t) [N]" "Fz_prop(t) [N]" \
-t "propulsive force"

python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/M_prop.csv" -s "$PLOT_DIR_PATH" \
-l "Mx_prop(t) [Nm]" "My_prop(t) [Nm]" "Mz_prop(t) [Nm]" \
-t "propulsive moment"

if [ -f "$DATA_DIR_PATH/windB.csv" ]; then
	python "$PLOT_2D_PATH" -d "$DATA_DIR_PATH/windB.csv" -s "$PLOT_DIR_PATH" \
	-l "windB_x(t) [m/s]" "windB_y(t) [m/s]" "windB_z(t) [m/s]" \
	-t "wind"
fi

python "$PLOT_3D_PATH" -d "$DATA_DIR_PATH/p.csv" -s "$PLOT_DIR_PATH" \
-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
-t "position"
