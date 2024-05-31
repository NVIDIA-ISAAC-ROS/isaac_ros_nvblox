# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os
import time

import rclpy
from rclpy.client import Client


def check_service_availability(
    instance: object,
    service_client: Client,
    service_name: str,
    timeout: float,
) -> None:
    """
    Check the availability of a service within a given timeframe.

    Parameters
    ----------
    instance : object
        An instance of the class invoking the method.

    service_client : Client
        An instance of a ROS 2 service client used to communicate with a service.

    service_name : str
        It should be a string representing the name of the ROS 2 service.

    timeout : float
        The timeout duration, specified in seconds

    """
    END_TIME = time.time() + timeout
    while not service_client.wait_for_service(timeout_sec=1.0):
        instance.assertLess(time.time(), END_TIME,
                            f'Timeout occurred while waiting for the {service_name} service')
        instance.node.get_logger().info(f'{service_name} service not available, waiting again...')


def get_service_response(instance: object, service_client: Client, service_request: object,
                         service_name: str, timeout: float) -> object:
    """
    Get the response from a service within a specified timeout.

    Parameters
    ----------
    instance : object
        An instance of the class invoking the method.

    service_client : Client
        The client object used to send the service request.

    service_request : object
        The request object to be sent to the service.

    service_name : str
        It should be a string representing the name of the ROS 2 service.

    timeout : float
        The timeout duration, specified in seconds

    Returns
    -------
    object
        The response object returned by the service,
        or None if the response is not received within the specified timeout period.

    """
    future_object = service_client.call_async(service_request)
    rclpy.spin_until_future_complete(instance.node, future_object, timeout_sec=timeout)
    service_response = future_object.result()
    instance.assertNotEqual(
        service_response, None,
        f'Timeout occurred while waiting for a response from {service_name} service.')
    return service_response


def is_service_succeeded(instance: object, service_response: object, service_name: str,
                         file_path: str) -> bool:
    """
    Get the response from a service within a specified timeout.

    Parameters
    ----------
    instance : object
        An instance of the class invoking the method.

    service_response : object
        The response object returned by the service.

    service_name : str
        It should be a string representing the name of the ROS 2 service.

    file_path : str
        The file path to check for the presence of a file.

    Returns
    -------
    bool
        True if the service call is considered successful based on the response, otherwise False.

    """
    done = False
    if service_response.success and os.path.exists(file_path):
        instance.assertGreater(
            os.path.getsize(file_path), 0, f'File size is reported as 0 bytes, '
            f'indicating an empty {os.path.basename(file_path)} file')
        done = True

    log_msg = (f'{service_name} service response '
               f'(success:{service_response.success})')
    instance.node.get_logger().info(log_msg)
    return done
