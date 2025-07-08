# tulip_user_peripherals

This repository illustrates how you can use custom peripherals with [airo-tulip](https://github.com/airo-ugent/airo-tulip).
Specifically, it shows how to use flow and compass sensors, connected to a Teensy 4.1, to compute odometry for the robot.

# Scripts

- main.py: The main script that compares odometry with the drive encoders and the flow sensors.
- test_flow.py: A script to test the flow sensors: drives the robot forward and prints the flow sensor readings.

## Installation

Make sure you have `uv` installed: see [astral-sh's documentation](https://github.com/astral-sh/uv#installation).

Then, install the dependencies: `uv sync`.

## Usage

Run the main script with `uv run python main.py`.  
You can also run the test script with `uv run python test_flow.py`.
