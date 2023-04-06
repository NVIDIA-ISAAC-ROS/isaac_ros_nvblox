# Troubleshooting ROS 2 communication issues

If it looks like you are dropping messages or you are not receiving any messages, you might be at the right place here.

## DDS tuning

In case you encounter issues with reliability of ROS 2 message delivery,
you could try to increase the maximum receive buffer size:

```bash
sudo sysctl -w net.core.rmem_max=2147483647
```

For more information please consult the [DDS tuning information of ROS 2](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html).

## RTPS profile

You can find the rtps profile of Isaac ROS [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docker/middleware_profiles/rtps_udp_profile.xml).
If you encounter communication issues when launching many ROS 2 nodes simultaneously you might want to increase the `maxInitialPeersRange` parameter like shown below:

```xml
<license>NVIDIA Isaac ROS Software License</license>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
            <maxInitialPeersRange>100</maxInitialPeersRange>
        </transport_descriptor>
    </transport_descriptors>

...
```

After saving the modification, make sure to restart you docker container.
