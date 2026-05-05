# flight-sim

C++ flight simulator with FlightGear integration

## Setup
- Install FlightGear: https://www.flightgear.org/download/

- Clone the repository
  ```bash
  git clone git@github.com:derekc22/flight-sim.git
  ```

- Create Conda environment:
  ```bash
  conda env create -f environment.yml
  conda activate ame532
  ```

- Install C++ dependencies:
  ```bash
  # Ubuntu
  sudo apt install libeigen3-dev libcppad-dev nlohmann-json3-dev libslicot-dev gfortran

  # macOS
  brew install eigen cppad nlohmann-json slicot gcc
  ```

## Usage

- The simulator is configured via JSON files in the `config/` directory
- `config/run.json` selects which JSON configuration files are active for a given run
- `.env` stores local machine paths for the repository, FlightGear, and the FlightGear protocol directory
- `scripts/run.sh` is the main entry point. It handles the normal run workflow, including launching FlightGear and running the simulator.
- Run `scripts/run.sh -h` to see the available command-line options

## Run

- Update the following environment variables in `.env` with the corresponding paths on your machine:
  - `DIR`: Project root
  - `SRC`: Project `schema` directory
  - `DEST`: FlightGear `Protocol` directory 
  - `FG`: FlightGear executable
  
- Update the following JSON configuration file paths in `config/run.json`:
  - `initialization_config`: Initial conditions
  - `structural_config`: Structural properties
  - `aerodynamics_config`: Aerodynamic properties
  - `control_config`: Control properties
  - `guidance_config`: Guidance properties
  - `estimation_config`: Estimation properties
  - `avionics_config`: Avionics properties
  - `actuator_config`: Actuator properties
  - `operating_config`: Operating properties
  
- To run the simulation: `scripts/run.sh -a <AIRCRAFT> -t <TIME_SEC> [-r <TRIM>] [-s <SENSOR>] [-c <CONTROL>] [-e <ESTIMATION>] [-v <VERBOSE>] [-d <DATA>] [-o <OUT_DIR>] [-p <PLOT>]`
  - `-a AIRCRAFT`: Aircraft to simulate (e.g. ASW28, c172p)
  - `-t TIME_SEC`: Simulation time in seconds
  - `-r TRIM`: Enable trim, linearization, and eigenanalysis
  - `-c CONTROL`: Enable control system
  - `-e ESTIMATION`: Enable state estimation
  - `-s SENSOR`: Enable sensors and avionics
  - `-v VERBOSE`: Enable verbose output
  - `-d DATA`: Enable data logging
  - `-p PLOT`: Enable plotting
  - `-o OUT_DIR`: Output directory
  
## Example
Run a 60-second simulation of the C172 with control and avionics. Outputs are saved to `data/c172_test` and `plots/c172_test`
```bash
scripts/run.sh -a c172p -t 60 -c -s -d -p -o c172_test
```

## References:
https://hopsan.github.io/tutorials/tutorial_flightgear.pdf

https://wiki.flightgear.org/Property_browser

https://github.com/c172p-team/c172p
