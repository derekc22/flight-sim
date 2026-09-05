import argparse
from pathlib import Path


def init_analysis():
    parser = argparse.ArgumentParser(description="Analysis CLI arguments")
    parser.add_argument("config_path", type=Path, help="Analysis config file path")
    parser.add_argument("data_dir_path", type=Path, help="Data directory path")
    parser.add_argument("plot_dir_path", type=Path, help="Plot directory path")
    parser.add_argument("report_dir_path", type=Path, help="Report directory path")
    args = parser.parse_args() 

    return args
