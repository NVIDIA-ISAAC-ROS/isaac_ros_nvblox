#!/bin/bash
mkdir -p ${WORKSPACE}/test_results
colcon test-result --all --result-files-only | xargs -I '{}' cp '{}' -t ${WORKSPACE}/test_results
