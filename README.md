# Carla Examples

This repositroy contains several car driving examples on Carla
simulator. It mainly uses Carla's [Python
API](https://carla.readthedocs.io/en/latest/python_api/). This
following examples are included:

- [follow\_a\_car](follow_a_car/README.md)
- [mountain\_driving](mountain_driving/README.md)
- [drive\_and\_log](drive_and_log/README.md)

## Setup

This repositroy was tested on Ubuntu 20.04 with Python 3.8. A graphics
card is recommended for fast and fluent simulation.


**CARLA 0.9.14** simulator is required to run the simlation
environment.

1. Visit Carla's GitHub release page
   [here](https://github.com/carla-simulator/carla/releases). Download
   `CARLA_0.9.14.tar.gz` and `AdditionalMaps_0.9.14.tar.gz`.
2. Extract the downloaded tarballs to the the `carla-simulator`
   directory.

**Poetry 1.3** is requried to launch the examples in this repo.

1. Visit [poetry.org doc](https://python-poetry.org/docs/) and follow
   the instructions in the page to install `poetry`.
2. After installation, your terminal must recognize the `poetry`
   command.

## Usage

The general steps go in two parts: launch the simulator server and
launch the client for a specific example. The server must run all the
time. To run an exmaple, choose an example directory in this repo and
launch the respective client.

To run the simulator server,

1. Open a terminal and check out to the ``carla-simulator``. Run
   `./CarlaUE4.sh` to start the simulator server.

To run an example scenario, say `follow_a_car`,

1. Open a terminal, go to the [follow\_a\_car](follow_a_car/README.md)
   directory.
3. Run `make` to launch the client. The client will configure the
scenario and apply car control.

## Architecture

These examples follow the simple server-client architecture, where the
Carla simulator acts as the server, while the client is a Python
script. The client uses [Python
API](https://carla.readthedocs.io/en/latest/python_api/) to control
the scene and cars on the server. You can read [this
tutorial](https://carla.readthedocs.io/en/latest/foundations/) to get
started.
