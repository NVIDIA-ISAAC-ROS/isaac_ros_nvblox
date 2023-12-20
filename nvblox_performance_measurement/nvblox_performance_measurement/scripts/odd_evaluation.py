#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import run_benchmark
import subprocess
import signal
import time
import os
import fitz
import rclpy
from PIL import Image


class ESDFSaver:

    def __init__(self):
        self.process = None

    def launch(self, folder_path: str, file_name: str):
        self.process = subprocess.Popen(
            f"exec ros2 launch nvblox_performance_measurement esdf_map_saver.launch.py file_name:={file_name} folder_path:={folder_path}",
            shell=True,
            preexec_fn=os.setsid,
        )

    def kill(self):
        if self.process is not None:
            print("killing esdf map saver")
            self.process.kill()
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)


def create_eval_results(setup_dir: str):
    """The function which takes in the template pdf and updates it with the output cost maps

    Args:
        setup_dir (str): The directory which contains the output cost maps and the template pdf file
    """
    # Open the existing PDF
    pdf_document = fitz.open(os.path.join(setup_dir, "odd_eval_template.pdf"))

    # List all image files
    image_files = [
        f for f in os.listdir(setup_dir)
        if f.endswith((".png", ".jpg", ".jpeg"))
    ]

    # Image placement dictionary. The position of the output files in the odd eval template
    image_placement_dict = {
        "obstacles_at_3m_dist.png": (590, 385, 150, 150, 0),
        "obstacles_at_5m_dist.png": (590, 280, 150, 150, 0),
        "obstacles_at_7m_dist.png": (590, 175, 150, 150, 0),
        "forklift_tine_on_ground.png": (590, 410, 150, 150, 1),
        "forklift_tine_on_ground_close.png": (590, 305, 150, 150, 1),
        "forklift_tine_elevated.png": (590, 200, 150, 150, 1),
        "pipe_on_bucket.png": (590, 95, 150, 150, 1),
    }

    for image_file in image_files:
        image_path = os.path.join(setup_dir, image_file)

        with Image.open(image_path) as img:
            mirrored_img = img.transpose(Image.FLIP_TOP_BOTTOM)
            temp_image_path = "temp_" + image_file
            temp_image_path = os.path.join(setup_dir, temp_image_path)
            mirrored_img.save(temp_image_path)
        # Determine image placement on the page
        if image_file in image_placement_dict:
            x, y, width, height, page_number = image_placement_dict[image_file]
        else:
            x, y, width, height, page_number = (
                0,
                0,
                1,
                1,
                0,
            )  # Dummy values to paste a very small image which cant be seen
        # Load the specific page
        page = pdf_document[page_number]
        # Add the image
        page.insert_image(fitz.Rect(x, y, x + width, y + height),
                          filename=temp_image_path)

        os.remove(temp_image_path)

    # Save the modified PDF to a new file
    output_pdf_path = os.path.join(setup_dir, "odd_eval_results.pdf")
    pdf_document.save(output_pdf_path)


def main(args=None):
    """Main function which gets in the arguments, runs the bags with the pipelines and calls the function to update the template

    Args:
        args (argparse, optional): The arguments to take in for the argparser. Defaults to None.
    """
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        description="Run ODD evaluation on nvblox_ros.")
    parser.add_argument(
        "bag_folderpath",
        required=True,
        nargs="?",
        help=
        "Path to the folder which contains the odd bag to run the benchmark on.",
    )
    parser.add_argument(
        "--setup_dir", help="Directory which contains the template files etc.")
    parser.add_argument(
        "--bag_play_args",
        default="",
        help='A (quotation bounded) list of bag replay arguments. E.g.:'
        'bag_play_args:="--remap /old_topic:=/new_topic --qos-profile-overrides-path qos_overrides.yaml"',
    )

    args = parser.parse_args()

    # Initialize all the required classes
    bag_launcher = run_benchmark.BagLauncher()
    esdf_saver = ESDFSaver()
    nvblox_runner = run_benchmark.NvbloxRunner(
        "hawk",
        True,  # So we dont run the results collector node
        False,  # So we dont run the dynamics
        "model_name_placecard",
        "model_repository_paths_placecard",
        "input_binding_name_placecard",
        "output_binding_name_placecard",
    )

    for bag in os.listdir(args.bag_folderpath):
        print("Launching pipelines")
        nvblox_runner.launch()
        print(
            "Waiting for: 15 sec for nvblox to start up before launching bags."
        )
        time.sleep(15)
        bag_path = os.path.join(args.bag_folderpath, bag)
        print(f"{bag} Bag is launching")
        esdf_saver.launch(args.setup_dir, str(bag) + ".png")
        time.sleep(3)
        bag_launcher.launch(bag_path, args.bag_play_args)
        print("bag done")
        print("Killing the running pipelines")
        esdf_saver.kill()
        nvblox_runner.kill()
        time.sleep(15)

    # Update the template with the generated output cost maps
    create_eval_results(args.setup_dir)


if __name__ == "__main__":
    main()
