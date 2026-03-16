python3 scripts/subplot.py -d example/p   -l "x(t) [m]"      "y(t) [m]"       "z(t) [m]"     -t "position vs time";
python3 scripts/subplot.py -d example/eul -l "psi(t) [rad]"  "theta(t) [rad]" "phi(t) [rad]" -t "orientation vs time";
python3 scripts/subplot.py -d example/w   -l "p(t) [rad/s]"  "q(t) [rad/s]"   "r(t) [rad/s]" -t "angular velocity vs time";
python3 scripts/subplot.py -d example/v   -l "vx(t) [m/s]"   "vy(t) [m/s]"    "vz(t) [m/s]"  -t "linear velocity vs time";