# self-racing-cars

## Setup

### Catkin workspace

Set up the catkin workspace:
* Create a folder for the workspace, for instance: `mkdir catkin_ws`
* Clone this repo in the workspace, as `src`: `cd catkin_ws && git clone https://github.com/roux-antoine/self-racing-cars.git src`

### Virtual environment

Create the virtual environment:
* Create a directory called `venv` at the root of the repo: `mkdir venv`
* Create a virtual environment in this folder: `python3 -m venv venv`
* Activate the virtual environment: `source venv/bin/activate`
* Install the python packages: `pip3 install -r requirements.txt`. Note: on the Pi I had issues because the package `setuptools` has a version too low (44.0.0), but apparently the installation still recovered by itself.

Before running any python code, remember to activate the virtual environment

The virtual environment can be exited by running: `deactivate`

Remember to re-run the `pip3 install -r requirements.txt` every time a new package is added to `requirements.txt`

TODO: figure out why scripts started with rosrun don't find python packages from the virtual env

### Install pre-commit hooks

To install the pre-commit hooks, run: `pre-commit install`

It is possible to commit without running the commit hooks by running `git commit --no-verify`

## Building the `arduino_pkg` package

Follow the instruction in `arduino_pkg/README.md`