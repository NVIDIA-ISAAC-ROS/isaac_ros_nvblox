#!/usr/bin/env python3
# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import argparse
from typing import List, Dict
from os import PathLike

import pathlib
import numpy as np
import pandas as pd
import glob

from nvblox_evaluation.evaluation_utils import parse_nvblox_timing


def read_tables_from_ros_log(
        logs_path: PathLike,
        start_delimiter: str,
        end_delimiter: str,
        table_name_to_column_index: Dict[str, int],
        table_start_row: int,
        remove_short_tables: bool = True,
        min_lines_between_delimiters: int = 3) -> List[pd.DataFrame]:
    """Scrapes an nvblox ros log and returns the rates tables it finds.

    Args:
        logs_path (PathLike): Path to the ros logs.
        remove_short_tables (bool, optional): Remove tables with less entries than the maximum
                                              number. Defaults to True.
        min_lines_between_delimiters (int, optional): _description_. Defaults to 3.

    Returns:
        List[pd.DataFrame]: A list of DataFrames containing rates measurements. One entry in the
                            list for each time nvblox ros node dumped a table to the log.

    """
    with open(logs_path, 'r') as f:
        # Read files in file
        lines = f.readlines()
        # Find start and end points of rates tables
        started = False
        rates_table_start_idx = -1
        rates_table_end_idx = -1
        rates_table_start_indices = []
        rates_table_end_indices = []
        for idx, line in enumerate(lines):
            if start_delimiter in line:
                started = True
                rates_table_start_idx = idx
            if started and end_delimiter in line and \
                    ((idx - rates_table_start_idx) > min_lines_between_delimiters):
                rates_table_end_idx = idx
                rates_table_start_indices.append(rates_table_start_idx)
                rates_table_end_indices.append(rates_table_end_idx)
                started = False
        assert (len(rates_table_start_indices) == len(rates_table_end_indices))
        if not rates_table_start_indices or not rates_table_end_indices:
            print("Couldn't find any tables")
            return []
        # Extract tables (each table stored as a string)
        tables_str = []
        for start_idx, end_idx in zip(rates_table_start_indices,
                                      rates_table_end_indices):
            tables_str.append(''.join(lines[start_idx:end_idx + 1]))
        # Kick out tables which have less entries
        if remove_short_tables:
            tables_lengths = np.array(
                [len(table_str.splitlines()) for table_str in tables_str])
            keep_indices = np.where(
                tables_lengths == np.max(tables_lengths))[0]
            tables_str_new = []
            for idx in keep_indices:
                tables_str_new.append(tables_str[idx])
            tables_str = tables_str_new
        # Convert to pandas tables
        tables = []
        for table_str in tables_str:
            tables.append(
                parse_nvblox_timing.get_table_as_dataframe_from_string(
                    table_str,
                    table_name_to_column_index,
                    start_row=table_start_row))
        return tables


def read_rate_tables_from_ros_log(
        logs_path: PathLike,
        remove_short_tables: bool = True,
        min_lines_between_delimiters: int = 3) -> List[pd.DataFrame]:
    start_delimiter = 'NVBlox Rates'
    end_delimiter = '-----------'
    table_name_to_column_index = {'num_samples': 1, 'mean': 2}
    table_start_row = 3
    return read_tables_from_ros_log(
        logs_path,
        start_delimiter,
        end_delimiter,
        table_name_to_column_index=table_name_to_column_index,
        table_start_row=table_start_row,
        remove_short_tables=remove_short_tables,
        min_lines_between_delimiters=min_lines_between_delimiters)


def read_timer_tables_from_ros_log(
        logs_path: PathLike,
        remove_short_tables: bool = True,
        min_lines_between_delimiters: int = 3) -> List[pd.DataFrame]:
    start_delimiter = 'NVBlox Timings'
    end_delimiter = '-----------'
    table_name_to_column_index = {
        'num_calls': 1,
        'total_time': 2,
        'mean': 3,
        'std': 5,
        'min': 6,
        'max': 7
    }
    table_start_row = 3
    return read_tables_from_ros_log(
        logs_path,
        start_delimiter,
        end_delimiter,
        table_name_to_column_index=table_name_to_column_index,
        table_start_row=table_start_row,
        remove_short_tables=remove_short_tables,
        min_lines_between_delimiters=min_lines_between_delimiters)


def get_default_ros_log_dir() -> pathlib.Path:
    return pathlib.Path.home() / ".ros" / "log"


def get_latest_ros_logs_path(
    component_name: str, log_dir: PathLike = get_default_ros_log_dir()
) -> pathlib.Path:
    log_files = glob.glob(str(log_dir / (component_name + "*")))
    if not log_files:
        raise FileNotFoundError("No log files found.")
    log_file_indices = sorted([
        int(pathlib.Path(log_file).name.split("_")[3])
        for log_file in log_files
    ])
    latest_log_file_idx = log_file_indices[-1]
    log_files = glob.glob(
        str(log_dir / (component_name + "*" + str(latest_log_file_idx) + "*")))
    assert len(log_files) == 1
    log_file_path = pathlib.Path(log_files[0])
    assert log_file_path.exists()
    return log_file_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=
        "Plots timer and rate histories by scraping them from a ROS log.")
    parser.add_argument(
        "ros_log_path",
        type=pathlib.Path,
        help=
        "Path to the ROS logs file for the component container containing nvblox."
    )
    parser.add_argument(
        "--dont-show",
        action="store_const",
        dest="show",
        const=False,
        default=True,
        help="Flag indicating if we should show the plots.",
    )
    parser.add_argument("--output-dir",
                        type=pathlib.Path,
                        help="Path to directory to save figures in.")
    return parser.parse_args()


def main():
    args = parse_args()
    print("Scraping the logs for nvblox performance data.")
    rate_dfs = read_rate_tables_from_ros_log(args.ros_log_path)
    timer_dfs = read_timer_tables_from_ros_log(args.ros_log_path)
    print(
        f"Found {len(rate_dfs)} rate tables, and {len(timer_dfs)} timer tables."
    )
    print("Plotting.")
    import interpret_benchmark_results
    interpret_benchmark_results.plot_rate_histories(rate_dfs, timer_dfs,
                                                    args.show, args.output_dir)


if __name__ == "__main__":
    main()
