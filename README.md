# flight-sim

Flight simulator built in C++ with FlightGear integration

## Installation
https://www.flightgear.org/download/

## Usage
- Clone the repository

- Edit the following environment variables in `.env`:
  - `DIR`: Project path on your machine
  - `SRC`: Path to the project 'config' folder on your machine
  - `DEST`: Path to the FlightGear 'Protocol' folder on your machine
  - `FG`: Path to the FlightGear executable on your machine
  
- Run `conda env create -f environment.yml` followed by `conda activate ame532` to initialize the Python environment
  
- Run `scripts/run.sh` from the project root to start the simulator

- Run `python3 scripts/subplot.py -d <data> -e <extension> -l <labels> -t <title>` to visualize data

## References:
https://hopsan.github.io/tutorials/tutorial_flightgear.pdf

https://wiki.flightgear.org/Property_browser