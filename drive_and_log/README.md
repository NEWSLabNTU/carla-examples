# drive\_and\_log

This example creates a car automatically drives along a path. Users
can press `r` during simulation to start recording camera images into
files, or run with a `--record-on-start` option to enable recording
when the simulation starts.

## Launch the Simulation

1. Please follow the instructions
   [here](https://python-poetry.org/docs/) to install `poetry`. It is
   a Python package manager that downloads dependencies for this
   example.

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
key to start recording camera images. It store image files and vehicle
poses in an output directory named by a date like
`2023-03-03_22-23-00_output`.

Alternatively, pass `--record-on-start` option to enable recording by
the time the simulation starts.

```sh
poetry run main
```

## The Recorded Data

The output directory layout looks like this. The inner
`transform_log.csv` stores a sequence of vehicle poses in CSV. The
`image` directory stores camera images named in timestamps.

```
2023-03-03_22-23-00_output/
├── images/
│   ├── 00016640.png
│   ├── 00016641.png
│   ├── 00016642.png
│   ├── 00016643.png
│   └── ...
└── transform_log.csv
```

The image file names corresponds the `timestamp` values in CSV. For
example,

```csv
frame,timestamp,x,y,z,pitch,yaw,roll
00016640,2.4639113899320364,-52.13356018066406,-40.1802978515625,0.36480003595352173,0.0,90.43230438232422,0.0
00016641,2.5139113906770945,-52.13356018066406,-40.1802978515625,0.27660003304481506,0.0,90.43230438232422,0.0
...
```

This command converts image files to a video file. To do so, find the
first timestamp in the CSV, which is 16640 in the example. Fill in the
timestamp to the `-start_number` option. Also, change the output
directory name in `-i` option.

```sh
ffmpeg \
    -start_number 16640 \
    -framerate 1 \
    -i 2023-03-03_22-32-50_output/images/%08d.png \
    -c:v libx264 \
    -r 10 \
    output.mp4
```


Note that the coordinates are stored in left-hand rule and x-forward,
y-right and z-up convention.

## Configuration

The source file [`drive_and_log/config.py`](drive_and_log/config.py)
configures the map and the driving path for the simulation.

The `WORLD` variable to configure the map to be used. You can
check the official document
[here](https://carla.readthedocs.io/en/latest/core_map/) to look for a
list of built-in maps.

The `NPC3_ROUTE` variable is a list of
[`carla.Transofrm`](https://carla.readthedocs.io/en/latest/python_api/#carlatransform)
the configures the driving path for the vehicle. To record a new path
on your own, you can start the simulation and press `p` to disable
autopilot. Then, press `h` to show the help prompt to learn how to
control the car.


```python
# Set your world here.
WORLD = "Town10HD"

NPC3_ROUTE = [
    Transform(Location(x=-52.133560, y=-40.180298, z=0.482400), Rotation(pitch=0.000000, yaw=90.432304, roll=0.000000)),
    Transform(Location(x=-81.243576, y=12.964413, z=0.000293), Rotation(pitch=-0.000369, yaw=-179.756287, roll=-0.000031)),
    Transform(Location(x=-52.330223, y=-9.467075, z=0.000294), Rotation(pitch=-0.000061, yaw=89.905533, roll=0.000000)),
    Transform(Location(x=-103.806343, y=52.093533, z=0.000292), Rotation(pitch=-0.004945, yaw=-88.720642, roll=-0.000214)),
    Transform(Location(x=-72.047188, y=27.981750, z=0.000293), Rotation(pitch=-0.000396, yaw=0.196186, roll=0.000000)),
    Transform(Location(x=-72.612579, y=128.264389, z=0.001414), Rotation(pitch=-0.004337, yaw=-164.806107, roll=-0.474091)),
]
```
