# Flight Simulator

A flight simulator built in C++

## Installation
https://www.flightgear.org/download/

## Usage
Clone the repository

Change `DIR` in `scripts/run.sh` to the project path on your machine

Change `SRC` in `scripts/write_in_xml.sh` and `scripts/write_out_xml.sh` to the project path on your machine

Change `DEST` in `scripts/write_in_xml.sh` and `scripts/write_out_xml.sh` to the FlightGear 'Protocol' folder path on your machine

Change `FG` in `scripts/launch.sh` to the FlightGear executable path on your machine

For plotting, create and install the `ame532` Conda environment by running `conda env create -f environment.yml` followed by `conda activate ame532`

Run `scripts/run.sh`

## References:
https://hopsan.github.io/tutorials/tutorial_flightgear.pdf

https://wiki.flightgear.org/Property_browser