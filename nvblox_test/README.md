# nvblox_tests

The integration tests in this package run tests for our top level launch files to check if the pipeline produces mesh and esdf messages.

# Running a test

Build the package using
```bash
colcon build --symlink-install --packages-up-to nvblox_test
```

Run the tests using
```bash
colcon test --event-handlers console_direct+ --packages-select nvblox_test
```
