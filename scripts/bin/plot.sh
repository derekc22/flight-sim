#!/bin/bash
set -e

CONDA_BASE=$(conda info --base)
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate ame532

PLOT_2D="scripts/lib/plot/plot_2d.py"
PLOT_3D="scripts/lib/plot/plot_3d.py"

if [ -z "$1" ]; then
	echo "Usage: $0 <DATA_DIR>"
	exit 1
fi

DATA_DIR="$1"
DATA_PATH="data/$DATA_DIR"

python "$PLOT_2D" -d "$DATA_DIR/p.csv" \
-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
-t "position vs time"

if [ -f "$DATA_PATH/p_meas.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/p_meas.csv" -r "$DATA_DIR/p.csv" \
	-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
	-t "position measurement vs time"
fi

if [ -f "$DATA_PATH/p_est.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/p_est.csv" -r "$DATA_DIR/p.csv" \
	-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
	-t "position estimate vs time"
fi

if [ -f "$DATA_PATH/eul_setpoint.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/eul.csv" -r "$DATA_DIR/eul_setpoint.csv" \
	-l "psi(t) [rad]" "theta(t) [rad]" "phi(t) [rad]" \
	-t "orientation vs time"
else
	python "$PLOT_2D" -d "$DATA_DIR/eul.csv" \
	-l "psi(t) [rad]" "theta(t) [rad]" "phi(t) [rad]" \
	-t "orientation vs time"
fi

if [ -f "$DATA_PATH/eul_meas.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/eul_meas.csv" -r "$DATA_DIR/eul.csv" \
	-l "psi(t) [rad]" "theta(t) [rad]" "phi(t) [rad]" \
	-t "orientation measurement vs time"
fi

if [ -f "$DATA_PATH/eul_est.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/eul_est.csv" -r "$DATA_DIR/eul.csv" \
	-l "psi(t) [rad]" "theta(t) [rad]" "phi(t) [rad]" \
	-t "orientation estimate vs time"
fi

if [ -f "$DATA_PATH/w_setpoint.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/w.csv" -r "$DATA_DIR/w_setpoint.csv" \
	-l "p(t) [rad/s]" "q(t) [rad/s]" "r(t) [rad/s]" \
	-t "angular velocity vs time"
else
	python "$PLOT_2D" -d "$DATA_DIR/w.csv" \
	-l "p(t) [rad/s]" "q(t) [rad/s]" "r(t) [rad/s]" \
	-t "angular velocity vs time"
fi

if [ -f "$DATA_PATH/w_meas.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/w_meas.csv" -r "$DATA_DIR/w.csv" \
	-l "p(t) [rad/s]" "q(t) [rad/s]" "r(t) [rad/s]" \
	-t "angular velocity measurement vs time"
fi

if [ -f "$DATA_PATH/w_est.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/w_est.csv" -r "$DATA_DIR/w.csv" \
	-l "p(t) [rad/s]" "q(t) [rad/s]" "r(t) [rad/s]" \
	-t "angular velocity estimate vs time"
fi

if [ -f "$DATA_PATH/v_setpoint.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/v.csv" -r "$DATA_DIR/v_setpoint.csv" \
	-l "vx(t) [m/s]" "vy(t) [m/s]" "vz(t) [m/s]" \
	-t "linear velocity vs time"
else
	python "$PLOT_2D" -d "$DATA_DIR/v.csv" \
	-l "vx(t) [m/s]" "vy(t) [m/s]" "vz(t) [m/s]" \
	-t "linear velocity vs time"
fi

if [ -f "$DATA_PATH/v_meas.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/v_meas.csv" -r "$DATA_DIR/v.csv" \
	-l "vx(t) [m/s]" "vy(t) [m/s]" "vz(t) [m/s]" \
	-t "linear velocity measurement vs time"
fi

if [ -f "$DATA_PATH/v_est.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/v_est.csv" -r "$DATA_DIR/v.csv" \
	-l "vx(t) [m/s]" "vy(t) [m/s]" "vz(t) [m/s]" \
	-t "linear velocity estimate vs time"
fi

python "$PLOT_2D" -d "$DATA_DIR/u_surface.csv" \
-l "elevator(t) [rad]" "aileron(t) [rad]" "rudder(t) [rad]" "flap(t) [rad]" "spoiler(t) [rad]" \
-t "surface inputs vs time"

python "$PLOT_2D" -d "$DATA_DIR/u_propulsor.csv" \
-l "front(t) [N]" "left(t) [N]" "right(t) [N]" \
-t "propulsor inputs vs time"

python "$PLOT_2D" -d "$DATA_DIR/F_net.csv" \
-l "Fx_net(t) [N]" "Fy_net(t) [N]" "Fz_net(t) [N]" \
-t "net force vs time"

python "$PLOT_2D" -d "$DATA_DIR/M_net.csv" \
-l "Mx_net(t) [Nm]" "My_net(t) [Nm]" "Mz_net(t) [Nm]" \
-t "net moment vs time"

python "$PLOT_2D" -d "$DATA_DIR/F_aero.csv" \
-l "Fx_aero(t) [N]" "Fy_aero(t) [N]" "Fz_aero(t) [N]" \
-t "aerodynamic force vs time"

python "$PLOT_2D" -d "$DATA_DIR/M_aero.csv" \
-l "Mx_aero(t) [Nm]" "My_aero(t) [Nm]" "Mz_aero(t) [Nm]" \
-t "aerodynamic moment vs time"

python "$PLOT_2D" -d "$DATA_DIR/F_prop.csv" \
-l "Fx_prop(t) [N]" "Fy_prop(t) [N]" "Fz_prop(t) [N]" \
-t "propulsive force vs time"

python "$PLOT_2D" -d "$DATA_DIR/M_prop.csv" \
-l "Mx_prop(t) [Nm]" "My_prop(t) [Nm]" "Mz_prop(t) [Nm]" \
-t "propulsive moment vs time"

if [ -f "$DATA_PATH/windB.csv" ]; then
	python "$PLOT_2D" -d "$DATA_DIR/windB.csv" \
	-l "windB_x(t) [m/s]" "windB_y(t) [m/s]" "windB_z(t) [m/s]" \
	-t "wind vs time"
fi

python "$PLOT_3D" -d "$DATA_DIR/p.csv" \
-l "x(t) [m]" "y(t) [m]" "z(t) [m]" \
-t "position"
