#!/bin/bash

# Script to print all params for a given node in rst table format
#
# Notes:
#   * The node is expected to be launched separately
#   * ROS's parameter interface is slow, so expect this to take a while

[[ $# -ne 1 ]] && echo "Usage: $0 node_name" && exit 1
node_name=$1

# Get all ROS params from the node
params=$(ros2 param list "$node_name")

tmp_file=$(mktemp)

# We use @ as delimiter so we can tabelify this using the columns command
TABLE_SEPARATOR="========================================================================@===========@==================@============"

echo "$TABLE_SEPARATOR" >> "$tmp_file"
echo "ROS Parameter@Type@Default@Description" >> "$tmp_file"
echo "$TABLE_SEPARATOR" >> "$tmp_file"

for param in $params
do
     # Get parameter from ROS. Expected format:
     # Parameter name: NAME Type: integer Description: DESCRIPTION TEXT Constraints:
     details=$(ros2 param describe "$node_name" "$param")

     name=$(echo $details | awk '{print $3}')
     type=$(echo $details | awk '{print $5}')

     # The description text is extracted by removing a trailing string
     desc=$(echo $details | cut -d ' ' -f7-)
     desc=${desc%"Constraints:"}

     # Get default value
     default=$(ros2 param get "$node_name" "$name" | awk '{print $4}')

     # Reduce number of decimals for doubles
     [[ "$type" == "double" ]] && default=$(printf "%.3f\n" $default)

     # Print the stuff
     echo \`\`$name\`\`@\`\`$type\`\`@\`\`$default\`\`@$desc >> "$tmp_file"
done

echo "$TABLE_SEPARATOR" >> "$tmp_file"

cat "$tmp_file"  | column -t -s@

echo $tmp_file
#rm "$tmp_file"


