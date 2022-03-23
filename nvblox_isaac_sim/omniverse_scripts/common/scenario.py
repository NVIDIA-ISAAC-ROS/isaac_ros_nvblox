# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
import omni
from omni.isaac.kit import SimulationApp


def load_scenario(simulation_app: SimulationApp, scenario_path: str):
    # Locate /Isaac folder on nucleus server to load sample
    from omni.isaac.core.utils.nucleus import find_nucleus_server
    result, nucleus_server = find_nucleus_server()
    if result is False:
        carb.log_error(
            "Could not find nucleus server with /Isaac folder, exiting")
        simulation_app.close()
        exit()

    usd_path = nucleus_server + scenario_path
    omni.usd.get_context().open_stage(usd_path, None)

    # Wait two frames so that stage starts loading
    simulation_app.update()
    simulation_app.update()

    print("Loading stage...")
    from omni.isaac.core.utils.stage import is_stage_loading
    while is_stage_loading():
        simulation_app.update()
    print("Loading Complete")

    print("Loading stage units")
    from omni.isaac.core.utils.stage import get_stage_units
    stage_units_m = get_stage_units()
    print(f"stage_units: {stage_units_m}")
    return stage_units_m
