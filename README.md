# flight-sim

C++ flight simulator with FlightGear integration

## Setup
- Clone the repository

- Install FlightGear: https://www.flightgear.org/download/

- Create Conda environment:
  - `conda env create -f environment.yml`
  - `conda activate ame532`

- Install dependencies:
  - `sudo apt install libeigen3-dev libcppad-dev nlohmann-json3-dev` (Linux)
  - `brew install eigen cppad nlohmann-json` (macOS)

## Usage

- Edit the following environment variables in `.env`:
  - `DIR`: Project path on your machine
  - `SRC`: Path to the project `config` folder on your machine
  - `DEST`: Path to the FlightGear `Protocol` folder on your machine
  - `FG`: Path to the FlightGear executable on your machine
  
- Edit the following JSON configuration file paths in `config/run.json`:
  - `aerodynamics_config`: Path to the aerodynamic properties JSON
  - `structural_config`: Path to the structural properties JSON
  - `control_config`: Path to the control properties JSON
  - `avionics_config`: Path to the avionics properties JSON
  - `initialization_config`: Path to the initial conditions JSON
  
- To run the simulation: `scripts/run.sh -a <AIRCRAFT> -t <TIME_SEC> [-r <TRIM>] [-s <SENSOR>] [-c <CONTROL>] [-v <VERBOSE>] [-d <DATA>] [-o <OUT_DIR>] [-p <PLOT>]`
  - `-a AIRCRAFT`: Aircraft to simulate (e.g. ASW28)
  - `-t TIME_SEC`: Simulation time in seconds
  - `-r TRIM`: Enable trim, linearization, and eigenanalysis
  - `-s SENSOR`: Enable sensors and avionics
  - `-c CONTROL`: Enable control system
  - `-v VERBOSE`: Enable verbose output
  - `-d DATA`: Enable data logging
  - `-o OUT_DIR`: Output directory
  - `-p PLOT`: Enable automatic plotting

- To plot manually: `scripts/plot.sh <OUT_DIR>`
  - `OUT_DIR`: Ouput directory specified at simulation runtime

## References:
https://hopsan.github.io/tutorials/tutorial_flightgear.pdf

https://wiki.flightgear.org/Property_browser