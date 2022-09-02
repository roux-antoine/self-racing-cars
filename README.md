# self-racing-cars

## Setup

### Virtual environment

Create the virtual environment:
* Create a directory called `venv` at the root of the repo: `mkdir venv`
* Create a virtual environment in this folder: `python3 -m venv venv`
* Activate the virtual environment: `source venv/bin/activate`
* Install the python packages: `pip3 install -r requirements.py`

Before running any python code, remember to activate the virtual environment
The virtual environment can be exited by running: `deactivate`
Remember to re-run the `pip3 install -r requirements.txt` every time a new package is added to requirements.py


### Install pre-commit hooks

To install the pre-commit hooks, run: `pre-commit install`
