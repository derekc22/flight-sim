# flight-sim

Flight simulator built in C++ with FlightGear integration

## Setup
- Clone the repository

- Install FlightGear: https://www.flightgear.org/download/

- Create Conda environment:
  - `conda env create -f environment.yml`
  - `conda activate ame532`

- Install dependencies:
  - `conda install -c conda-forge casadi`
  - `sudo apt install libeigen3-dev nlohmann-json3-dev` (Linux)
  - `brew install eigen nlohmann-json` (macOS)

## Usage

- Edit the following environment variables in `.env`:
  - `DIR`: Project path on your machine
  - `SRC`: Path to the project 'config' folder on your machine
  - `DEST`: Path to the FlightGear 'Protocol' folder on your machine
  - `FG`: Path to the FlightGear executable on your machine
  
- Edit the following JSON configuration file paths in `config/run.json`:
  - `aerodynamics_config_path`: Path to the aerodynamic properties JSON
  - `structural_config_path`: Path to the structural properties JSON
  - `initial_conditions_config_path`: Path to the initial conditions JSON
  
- Run `scripts/run.sh <AIRCRAFT> <TIME_SEC>` from the project root to start the simulator
  - `AIRCRAFT`: Aircraft to simulate (e.g. ASW28)
  - `TIME_SEC`: Simulation time in seconds

- Run `python3 scripts/subplot.py -d <DATA> -e <EXTENSION> -l <LABELS> -t <TITLE>` to visualize data

## References:
https://hopsan.github.io/tutorials/tutorial_flightgear.pdf

https://wiki.flightgear.org/Property_browser