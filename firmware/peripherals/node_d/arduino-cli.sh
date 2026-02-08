#!/bin/bash
# Initialize valid environment variables
source ./arduino-cli.env

# Run Arduino CLI
arduino-cli compile --fqbn $ARDUINO_BOARD_fqbn .
