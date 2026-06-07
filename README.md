# flight-sim

C++ flight simulator with FlightGear integration

## Installation
Download FlightGear: 
```
https://www.flightgear.org/download/
```

Clone repository:
```bash
git clone git@github.com:derekc22/flight-sim.git
```

Create Conda environment:
```bash
conda env create -f environment.yml
conda activate ame532
```

Install C++ dependencies:
```bash
# Ubuntu
sudo apt install libeigen3-dev libcppad-dev nlohmann-json3-dev libslicot-dev gfortran libgtest-dev

# macOS
brew install eigen cppad nlohmann-json slicot gcc googletest
```
Install MATLAB Runtime:
```
https://www.mathworks.com/products/compiler/matlab-runtime.html
```
## Usage

- The simulator is configured via JSON files in the `config/` directory
- `config/run.json` selects which JSON configuration files are active for a given run
- `scripts/bin/run.sh` runs the simulation and launches FlightGear
- `scripts/bin/plot.sh` generates plots from logged data
- `scripts/bin/analyze.sh` runs post-run analysis with MATLAB
- `scripts/bin/test.sh` runs unit tests
- `.env` stores local machine paths for the repository, FlightGear, and the FlightGear protocol directory

## Setup

Update the following environment variables in `.env` with the corresponding paths on your machine:
- `PROJ_PATH`: Project root
- `PRTCL_PATH`: FlightGear `Protocol` directory
- `FG_PATH`: FlightGear executable
- `MATLAB_PATH`: MATLAB executable
  
Update the following JSON configuration file paths in `config/run.json`:
- `initialization_config`: Initial conditions
- `structural_config`: Structural properties
- `aerodynamics_config`: Aerodynamic properties
- `avionics_config`: Avionics properties
- `actuator_config`: Actuator properties
- `control_config`: Control properties
- `estimation_config`: Estimation properties
- `guidance_config`: Guidance properties
- `runtime_config`: Runtime properties

Update the JSON configuration file paths in `config/analyze.json`
  
## Run

To run the simulation:
  ```bash
  ./scripts/bin/run.sh -a <AIRCRAFT> -t <TIME_SEC> [-r TRIM] [-s SENSOR] [-c CONTROL] [-e ESTIMATION] [-w WIND] [-v VERBOSE] [-d DATA] [-p PLOT] -o <OUT_DIR> [-m ANALYSIS]
  ```
- `-a AIRCRAFT`: Aircraft to simulate (e.g. ASW28, c172p)
- `-t TIME_SEC`: Simulation time in seconds
- `-r TRIM`: Enable trim, linearization, and eigenanalysis
- `-s SENSOR`: Enable sensors and avionics
- `-c CONTROL`: Enable control system
- `-e ESTIMATION`: Enable state estimation
- `-w WIND`: Enable wind
- `-v VERBOSE`: Enable verbose output
- `-d DATA`: Enable data logging
- `-p PLOT`: Enable plotting
- `-o OUT_DIR`: Output directory name
- `-m ANALYSIS`: Enable post-run analysis 
  
## Example
Run a 60-second simulation of the C172 with control and avionics enabled. Data, plots, and reports are saved to `results/data/c172_test`, `results/figures/c172_test`, and `results/reports/c172_test`, respectively:
```bash
./scripts/bin/run.sh -a c172p -t 60 -c -s -d -p -o c172_test
```

## Plotting
To generate plots from logged data, use the `-p` flag with `run.sh` or run:
```bash
./scripts/bin/plot.sh <OUT_DIR>
```

## Analysis
Post-run analysis is implemented with MATLAB and configured via `config/analyze.json`. To run the analysis on logged data, use the `-m` flag with `run.sh` or run:

```bash
scripts/bin/analyze.sh <OUT_DIR>
```
The current analysis implementation requires an active MATLAB license. A planned transition to pre-compiled binaries will remove this requirement, requiring only the MATLAB Runtime instead

## Tests
Unit tests are implemented with Google Test:
```bash
# Run all tests
./scripts/bin/test.sh

# Run a specific test suite
./scripts/bin/test.sh -t <TEST_SUITE>

# See available test suites
./scripts/bin/test.sh -h
```

## References:
https://hopsan.github.io/tutorials/tutorial_flightgear.pdf

https://wiki.flightgear.org/Property_browser

https://github.com/c172p-team/c172p
