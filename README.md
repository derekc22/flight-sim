# flight-sim

C++ flight simulator with FlightGear integration

## Setup
- Clone the repository

- Install FlightGear: https://www.flightgear.org/download/

- Create the Conda environment:
  - `conda env create -f environment.yml`
  - `conda activate ame532`

- Install dependencies:
  - `sudo apt install libeigen3-dev libcppad-dev nlohmann-json3-dev libslicot-dev gfortran` (Ubuntu)
  - `brew install eigen cppad nlohmann-json slicot gcc` (macOS)

## Usage

- The simulator is configured via JSON files in the `config/` directory
- `config/run.json` selects which configuration JSON files are active for a given run
- `.env` stores local machine paths for the repo, FlightGear, and the FlightGear protocol directory.
- `scripts/run.sh` is the main entry point. It handles the normal run workflow, including launching FlightGear and running the simulator with the selected options
- Run `scripts/run.sh -h` to see the available command-line options

## Run

- Update the following environment variables in `.env` with the corresponding paths on your machine:
  - `DIR`: Project root
  - `SRC`: Project `config` directory
  - `DEST`: FlightGear `Protocol` directory 
  - `FG`: FlightGear executable
  
- Update the following JSON configuration file paths in `config/run.json`:
  - `initialization_config`: Initial conditions
  - `structural_config`: Structural properties
  - `aerodynamics_config`: Aerodynamic properties
  - `control_config`: Control properties
  - `avionics_config`: Avionics properties
  - `actuator_config`: Actuator properties
  
- To run the simulation: `scripts/run.sh -a <AIRCRAFT> -t <TIME_SEC> [-r <TRIM>] [-s <SENSOR>] [-c <CONTROL>] [-v <VERBOSE>] [-d <DATA>] [-o <OUT_DIR>] [-p <PLOT>]`
  - `-a AIRCRAFT`: Aircraft to simulate (e.g. ASW28)
  - `-t TIME_SEC`: Simulation time in seconds
  - `-r TRIM`: Enable trim, linearization, and eigenanalysis
  - `-s SENSOR`: Enable sensors and avionics
  - `-c CONTROL`: Enable control system
  - `-v VERBOSE`: Enable verbose output
  - `-d DATA`: Enable data logging
  - `-o OUT_DIR`: Output directory
  - `-p PLOT`: Enable plotting

## References:
https://hopsan.github.io/tutorials/tutorial_flightgear.pdf

https://wiki.flightgear.org/Property_browser

https://github.com/c172p-team/c172p
