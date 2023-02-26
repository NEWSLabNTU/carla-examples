# drive\_and\_log

This example creates a car automatically drives along a path. Users
can press `r` during simulation to start recording camera images into
files, or run with a `--record-on-start` option to enable recording
when the simulation starts.

## Launch to Simulation

1. Please follow the instructions [here](https://python-poetry.org/docs/)
to install `poetry`. It is a Python package manager that downloads
dependencies for this example.

2. Prepare a virtual environment and install required dependencies.

    ```sh
    poetry install
    ```

3. Launch the example.

    ```sh
    poetry run main
    ```

## Recording

The recording feature is disabled when the program starts. Press `r`
key to start recording camera images. It store image files in the
`_out` directory.

Pass `--record-on-start` option to enable recording by the time the
simulation starts.

```sh
poetry run main
```

## Configuration

The source file [`drive_and_log/config.py`](drive_and_log/config.py)
configures the map and the driving path for the simulation.

The `WORLD` variable to configure the map to be used. You can
check the official document
[here](https://carla.readthedocs.io/en/latest/core_map/) to look for a
list of built-in maps.

The `NPCx_ROUTE` variable is a list of
[`carla.Transofrm`](https://carla.readthedocs.io/en/latest/python_api/#carlatransform)
the configures the driving path for the vehicle. To record a new path
on your own, you can start the simulation and press `p` to disable
autopilot. Then, press `h` to show the help prompt to learn how to
control the car.


```python
WORLD = "Town03"

NPC1_ROUTE = [
    Transform(
        Location(13.695746, 18.811819, 0.1), Rotation(-0.000031, -43.884972, -0.007568)
    ),
    Transform(
        Location(22.825905, 1.144211, 0.1), Rotation(0.003324, -89.318871, -0.000676)
    ),
    # ...
]
```
