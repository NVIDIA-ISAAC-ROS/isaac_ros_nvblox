#!/usr/bin/env python3
# Copyright (c) 2023, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from typing import Optional, List, Dict
from os import PathLike
import pathlib
import argparse

import numpy.typing as npt
import pandas as pd
import matplotlib.pyplot as plt

from nvblox_evaluation.evaluation_utils import parse_nvblox_timing
import parse_ros_logs


def make_plot_fullscreen():
    """Makes the figure window fullscreen."""
    plt.get_current_fig_manager().full_screen_toggle()


def get_ros_timings_as_dataframe(filepath: PathLike) -> pd.DataFrame:
    """Load ros-relevant timers from a file and return them in a DataFrame.

    Args:
        filepath (PathLike): Path to .txt timers file

    Returns:
        pd.DataFrame: DataFrame containing timings

    """
    timings_df = parse_nvblox_timing.get_timings_as_dataframe(filepath)
    ros_row_flags = [
        timer.startswith("ros") for timer in timings_df.index.to_list()
    ]
    return timings_df[ros_row_flags]


def get_ros_rates_as_dataframe(filepath: PathLike) -> pd.DataFrame:
    """Load ros-relevant rates from a file and return them in a DataFrame.

    Args:
        filepath (PathLike): Path to .txt rates file

    Returns:
        pd.DataFrame: DataFrame containing rates

    """
    rates_df = parse_nvblox_timing.get_rates_as_dataframe(filepath)
    ros_row_flags = [
        timer.startswith("ros") for timer in rates_df.index.to_list()
    ]
    return rates_df[ros_row_flags]


def plot_ros_timings(
    ros_timings_df: pd.DataFrame,
    show: Optional[bool] = True,
    output_dir: Optional[pathlib.Path] = None,
) -> None:
    """Generate plot for ros timers.

    Args:
        ros_timings_df (pd.DataFrame): DataFrame containing timings to plot.
        show (Optional[bool], optional): Whether to call plt.show(). Defaults to True.
        output_dir (Optional[pathlib.Path], optional): Where to output plot. Defaults to None.

    """
    # We have long names for the bars so this makes them fit on the plot.
    plt.rcParams["figure.autolayout"] = True

    # Plot - Total time
    plt.figure()
    ros_timings_df["total_time"].plot(kind="bar")
    make_plot_fullscreen()
    plt.title("Total ROS Processing Times (s)")
    plt.ylabel("Total Time (s)")
    if output_dir:
        plt.savefig(output_dir / "ros_timers_total.png", bbox_inches="tight")
    if show:
        plt.show()
    plt.figure().clear()

    # Plot - Mean time
    plt.figure()
    ros_timings_df["mean"].plot(kind="bar")
    make_plot_fullscreen()
    plt.title("Mean ROS Processing Times (s)")
    plt.ylabel("Mean Time (s)")
    if output_dir:
        plt.savefig(output_dir / "ros_timers_mean.png", bbox_inches="tight")
    if show:
        plt.show()
    plt.figure().clear()


def plot_ros_rates(
    ros_rates_df: pd.DataFrame,
    show: Optional[bool] = True,
    output_dir: Optional[pathlib.Path] = None,
) -> None:
    """Generate plot for ros rates.

    Args:
        ros_timings_df (pd.DataFrame): DataFrame containing rates to plot.
        show (Optional[bool], optional): Whether to call plt.show(). Defaults to True.
        output_dir (Optional[pathlib.Path], optional): Where to output plot. Defaults to None.

    """
    # NOTE(alexmillane): This function depends on which backend is being used by matplotlib. I
    # can't be bothered making it general at the moment so it only works for the `TkAgg` backend
    # which is being used in dazel at the moment.

    # We have long names for the bars so this makes them fit on the plot.
    plt.rcParams["figure.autolayout"] = True

    # Plot - Total time
    plt.figure()
    ros_rates_df["mean"].plot(kind="bar")
    make_plot_fullscreen()
    plt.title("ROS Processing Rates (s)")
    plt.ylabel("Rate (Hz)")
    if output_dir:
        plt.savefig(output_dir / "ros_rates.png", bbox_inches="tight")
    if show:
        plt.show()
    plt.figure().clear()


def plot_message_timestamps(
    diffs_header_ms: npt.ArrayLike,
    diffs_system_ms: npt.ArrayLike,
    average_diffs_header_ms: npt.ArrayLike,
    average_diffs_system_ms: npt.ArrayLike,
    show: Optional[bool] = True,
    output_dir: Optional[pathlib.Path] = None,
) -> None:
    """Generate plot for message timestamps.

    Args:
        diffs_header_ms (npt.ArrayLike): Diffs between header stamps
        diffs_system_ms (npt.ArrayLike): Diffs between system stamps
        average_diffs_header_ms (npt.ArrayLike): Rolling average of header diffs
        average_diffs_system_ms (npt.ArrayLike): Rolling average of system diffs
        show (Optional[bool], optional): Whether to call plt.show(). Defaults to True.
        output_dir (Optional[pathlib.Path], optional): Where to output plot. Defaults to None.

    """
    # Plot diffs
    plt.plot(diffs_header_ms)
    plt.plot(diffs_system_ms)
    plt.plot(average_diffs_header_ms)
    plt.plot(average_diffs_system_ms)
    plt.xlabel("Message Index")
    plt.ylabel("Time Diffs (ms)")
    plt.title("Inter-Message Time Diff")
    plt.legend([
        "Header stamps",
        "System stamps",
        "Header stamps (averaged)",
        "System stamps (averaged)",
    ])
    if output_dir:
        plt.savefig(output_dir / "message_timestamp_diffs.png")
    if show:
        plt.show()
    plt.figure().clear()


def tables_to_histories_map(tables: List[pd.DataFrame],
                            stat_name: str) -> Dict[str, List]:
    """Convert a list of tables to map of ticker names to entries.

    Args:
        tables (List[pd.DataFrame]): List of tables. Each table needs to contain the same entries.

    Returns:
        Dict[str, List]: A dict mapping ticker names to a history of measurements.

    """
    if len(tables) == 0:
        return {}
    # Construct a history of rates for each ticker
    ticker_histories = {}
    for ticker_name in tables[0].index:
        ticker_histories[ticker_name] = []
    for table in tables:
        stats = table[stat_name]
        for index in stats.index:
            ticker_histories[index].append(stats[index])
    return ticker_histories


def plot_rate_histories(
    rate_tables: List[pd.DataFrame],
    timer_tables: List[pd.DataFrame],
    show: Optional[bool] = True,
    output_dir: Optional[pathlib.Path] = None,
) -> None:
    """Plot the nvblox rate histories (the measured ticker rates over time).

    Args:
        rate_tables (List[pd.DataFrame]): Rates tables for for each measurement point
        rate_tables (List[pd.DataFrame]): Timer tables for for each measurement point
        show (Optional[bool], optional): Whether to call plt.show(). Defaults to True.
        output_dir (Optional[pathlib.Path], optional): Where to output plot. Defaults to None.

    """
    # Exclude timers not starting with "ros"
    ros_timer_tables = []
    for timer_table in timer_tables:
        ros_row_flags = [
            timer.startswith("ros") for timer in timer_table.index.to_list()
        ]
        ros_timer_tables.append(timer_table[ros_row_flags])
    timer_tables = ros_timer_tables

    # Convert the tables to per-ticker-histories
    timer_histories = tables_to_histories_map(timer_tables, "mean")
    rate_histories = tables_to_histories_map(rate_tables, "mean")

    plt.rcParams["figure.autolayout"] = True

    # Plot - Rates
    if rate_histories:
        for name, hist in rate_histories.items():
            plt.plot(hist, label=name)
        make_plot_fullscreen()
        plt.xlabel("Measurement index")
        plt.ylabel("Rate (Hz)")
        plt.yscale("log")
        plt.title("Measured Rate of nvblox Functions over Dataset")
        plt.legend()
        if output_dir:
            plt.savefig(output_dir / "ros_rate_histories.png")
        if show:
            plt.show()
        plt.figure().clear()

    # Plot - Timers
    if timer_histories:
        plt.figure(figsize=(20, 10))  # Big fig for huge legend
        for name, hist in timer_histories.items():
            plt.plot(hist, label=name)
        make_plot_fullscreen()
        plt.xlabel("Measurement index")
        plt.ylabel("Duration (s)")
        plt.title("Timers of nvblox Functions over Dataset")
        plt.legend()
        if output_dir:
            plt.savefig(output_dir / "ros_timer_histories.png")
        if show:
            plt.show()
        plt.figure().clear()


def plot_benchmarking_results(
    timers_filepath: Optional[pathlib.Path] = None,
    rates_filepath: Optional[pathlib.Path] = None,
    output_dir: Optional[pathlib.Path] = None,
    show: bool = True,
):
    """Plot results from a nvblox_ros benchmarking run."""

    if timers_filepath:
        if timers_filepath.exists():
            # Load the timers prepended with "ros"
            print(f"Loading timings at: {timers_filepath}")
            ros_timings_df = get_ros_timings_as_dataframe(timers_filepath)
            # Plot
            plot_ros_timings(ros_timings_df, show=show, output_dir=output_dir)
        else:
            print(f"The timing file path at {timers_filepath} does not exist")

    if rates_filepath:
        if rates_filepath.exists():
            # Load the rates prepended with "ros"
            print(f"Loading rates at: {rates_filepath}")
            ros_rates_df = get_ros_rates_as_dataframe(rates_filepath)
            # Plot
            plot_ros_rates(ros_rates_df, show=show, output_dir=output_dir)
        else:
            print(f"The rates file path at {rates_filepath} does not exist")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interprets the result of a benchmarking run.")
    parser.add_argument("--timers-filepath",
                        type=pathlib.Path,
                        help="Path to the timers file.")
    parser.add_argument("--rates-filepath",
                        type=pathlib.Path,
                        help="Path to the rates file.")
    parser.add_argument("--output-dir",
                        type=pathlib.Path,
                        help="Path to directory to save figures in.")
    parser.add_argument(
        "--dont-show",
        action="store_const",
        dest="show",
        const=False,
        default=True,
        help="Flag indicating if we should show the plots.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    plot_benchmarking_results(
        timers_filepath=args.timers_filepath,
        rates_filepath=args.rates_filepath,
        output_dir=args.output_dir,
        show=args.show,
    )
