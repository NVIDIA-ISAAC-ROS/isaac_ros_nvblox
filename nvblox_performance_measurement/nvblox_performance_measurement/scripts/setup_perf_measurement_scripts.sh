#!/bin/bash

# Paths to the local packages with setup.py
LOCAL_PACKAGES=(
    "/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox/python/common/"
    "/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox/python/evaluation/"
    "/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox/python/scripts/"
)

# Define standalone Python packages to install
STANDALONE_PACKAGES=(
    "matplotlib"
    "pandas"
    "pymupdf"
)

# Upgrade pip and setuptools
pip install -U pip setuptools

# Install standalone Python packages
echo "Installing standalone packages..."
for PACKAGE in "${STANDALONE_PACKAGES[@]}"; do
    echo "Installing $PACKAGE"
    pip install $PACKAGE
done

# Install local packages with setup.py
echo "Installing local packages..."
for PACKAGE_DIR in "${LOCAL_PACKAGES[@]}"; do
    echo "Installing package in $PACKAGE_DIR"
    cd $PACKAGE_DIR
    pip install .
    cd -  # Go back to the original directory
done

echo "All packages installed."
