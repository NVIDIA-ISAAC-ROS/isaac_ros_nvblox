# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import carb
import omni
from omni.isaac.kit import SimulationApp


def load_scenario(simulation_app: SimulationApp, scenario_path: str):
    # Locate /Isaac folder on nucleus server to load sample
    from omni.isaac.core.utils.nucleus import find_nucleus_server
    result, nucleus_server = find_nucleus_server()
    if result is False:
        carb.log_error(
            'Could not find nucleus server with /Isaac folder, exiting')
        simulation_app.close()
        exit()

    usd_path = nucleus_server + scenario_path
    omni.usd.get_context().open_stage(usd_path, None)

    # Wait two frames so that stage starts loading
    simulation_app.update()
    simulation_app.update()

    print('Loading stage...')
    from omni.isaac.core.utils.stage import is_stage_loading
    while is_stage_loading():
        simulation_app.update()
    print('Loading Complete')

    print('Loading stage units')
    from omni.isaac.core.utils.stage import get_stage_units
    stage_units_m = get_stage_units()
    print(f'stage_units: {stage_units_m}')
    return stage_units_m
