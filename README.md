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
sudo apt install pkg-config libeigen3-dev cppad nlohmann-json3-dev libslicot-dev gcc g++ libgtest-dev libopencv-dev

# macOS
brew install pkgconf eigen cppad nlohmann-json slicot gcc googletest opencv
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
- `.env` stores local machine paths for the repository, FlightGear, and MATLAB

## Setup

Update the following environment variables in `.env` with the corresponding paths on your machine:
- `PROJ_PATH`: Project root
- `FG_PATH`: FlightGear executable
- `MATLAB_PATH`: MATLAB executable
  
Customize the following JSON configuration file paths in `config/run.json`:
- `initialization_config`: Initial conditions
- `structural_config`: Structural properties
- `aerodynamics_config`: Aerodynamic properties
- `avionics_config`: Avionics properties
- `sensors_config`: Sensor properties
- `actuator_config`: Actuator properties
- `control_config`: Control properties
- `estimation_config`: Estimation properties
- `guidance_config`: Guidance properties
- `settings_config`: Module settings
- `runner_config`: Runner options

Customize the JSON configuration file paths in `config/analyze.json`
  
## Run

To run the simulation:
  ```bash
  ./scripts/bin/run.sh -a <AIRCRAFT> [-d DATA] [-p PLOT] [-m ANALYSIS] -o <OUT_DIR>
  ```
- `-a AIRCRAFT`: Aircraft to simulate (e.g. ASW28, c172p)
- `-d DATA`: Enable data saving
- `-p PLOT`: Enable plotting
- `-m ANALYSIS`: Enable post-run analysis 
- `-o OUT_DIR`: Output directory name
  
## Example
Run a simulation of the C172 with logs, data, plots, and reports saved to `results/logs/c172`, `results/data/c172`, `results/figures/c172`, and `results/reports/c172`, respectively:
```bash
./scripts/bin/run.sh -a c172p -d -p -m -o c172
```

## Plotting
To generate plots from logged data, use the `-p` flag with `run.sh` or run:
```bash
./scripts/bin/plot.sh <OUT_DIR>
```

## Rerun
To enable Rerun streaming, set `enable.rerun` to `true` in the `runner_config`. If the Rerun viewer fails to automatically launch, run:
```bash
rerun --connect rerun+http://127.0.0.1:9876/proxy
```
To use the recommended blueprint, drag and drop `assets/default.rbl` into the Rerun viewer or run:
```bash
rerun --connect rerun+http://127.0.0.1:9876/proxy assets/default.rbl
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
