# flight-sim

Flight simulator built in C++ with FlightGear integration

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
  - `SRC`: Path to the project 'config' folder on your machine
  - `DEST`: Path to the FlightGear 'Protocol' folder on your machine
  - `FG`: Path to the FlightGear executable on your machine
  
- Edit the following JSON configuration file paths in `config/run.json`:
  - `aerodynamics_config_path`: Path to the aerodynamic properties JSON
  - `structural_config_path`: Path to the structural properties JSON
  - `control_config_path`: Path to the control properties JSON
  - `initialization_config_path`: Path to the initial conditions JSON
  
- Run `scripts/run.sh -a <AIRCRAFT> -t <TIME_SEC> [-r <TRIM>] [-v <VERBOSE>] [-d <DATA>] [-f <FOLDER>]`
  - `-a AIRCRAFT`: Aircraft to simulate (e.g. ASW28)
  - `-t TIME_SEC`: Simulation time in seconds
  - `-r TRIM`: Enable trim
  - `-v VERBOSE`: Enable verbose output
  - `-d DATA`: Enable data logging
  - `-f FOLDER`: Folder to save data

- Run `scripts/plot.sh <FOLDER>`
  - `FOLDER`: Folder with saved data

## References:
https://hopsan.github.io/tutorials/tutorial_flightgear.pdf

https://wiki.flightgear.org/Property_browser