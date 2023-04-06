# How to get semantic GT from Isaac Sim

This package helps to convert semantic labels coming from Isaac Sim to consistent label images in mono8 and rgb8 used in nvblox.

# Nodes

`semantic_label_conversion.launch.py` launches the `semantic_label_stamper` and `semantic_label_converter` nodes.
The parameters in `params/semantic_label_conversion.yaml` are loaded.

## **semantic_label_stamper** node

Adds timestamps to the semantic labels. This is required to synchronize the semantic image and labels in the `semantic_label_converter` node with a `TimeSynchronizer`.

| Subscriber Topics                | Type                  | Description                                       |
|----------------------------------|-----------------------|---------------------------------------------------|
| `/<camera_name>/semantic_labels` | `std_msgs/msg/String` | Semantic labels definition published by Isaac Sim |

| Publisher Topics                                    | Type                                        | Description                           |
|-----------------------------------------------------|---------------------------------------------|---------------------------------------|
| `/semantic_conversion/<camera_name>/labels_stamped` | `semantic_labels/msg/SemanticLabelsStamped` | Resulting timestamped semantic labels |


## **semantic_label_converter** node

Outputs the resulting mono8 and rgb8 GT images.

| Subscriber Topics                                   | Type                                        | Description                                                                  |
|-----------------------------------------------------|---------------------------------------------|------------------------------------------------------------------------------|
| `/<camera_name>/semantic`                           | `sensor_msgs/msg/Image`                     | GT image published by Isaac Sim                                              |
| `/semantic_conversion/<camera_name>/labels_stamped` | `semantic_labels/msg/SemanticLabelsStamped` | Timestamped semantic labels published by the `semantic_label_converter` node |

| Publisher Topics                                        | Type                    | Description                             |
|---------------------------------------------------------|-------------------------|-----------------------------------------|
| `/semantic_conversion/<camera_name>/semantic_mono8`     | `sensor_msgs/msg/Image` | Resulting semantic GT one channel image |
| `/semantic_conversion/<camera_name>/semantic_colorized` | `sensor_msgs/msg/Image` | Resulting semantic GT colorized image   |


## Configuration

The configuration file can be found in `params/semantic_label_conversion.yaml`.

| Parameter                         | Type     | Description                                                                                                                             |
|-----------------------------------|----------|-----------------------------------------------------------------------------------------------------------------------------------------|
| `camera_1_enabled`                | `bool`   | Enable the conversion for camera 1 images                                                                                               |
| `camera_1_name`                   | `string` | Name of camera 1 (topic names are dependent on this parameter)                                                                          |
| `camera_2_enabled`                | `bool`   | Enable the conversion for camera 2 images                                                                                               |
| `camera_2_name`                   | `string` | Name of camera 2 (topic names are dependent on this parameter)                                                                          |
| `labels.names`                    | `list`   | List of strings containing label names defined in Isaac Sim that you wish to convert                                                    |
| `label.<label_name>.output_id`    | `int`    | Mono8 value that each label name gets converted to in the `/semantic_conversion/<camera_name>/semantic_mono8` image                     |
| `label.<label_name>.output_color` | `list`   | Rgb8 value stored as a list that each label name gets converted to in the `/semantic_conversion/<camera_name>/semantic_colorized` image |
