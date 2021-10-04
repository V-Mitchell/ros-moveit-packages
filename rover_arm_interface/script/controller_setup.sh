#!/bin/bash
# Author: Victor M
# Email: victorcmitchell@gmail.com

# Controller Setup Commands

REFRESH_RATE=10
DEAD_ZONE=0.075
DEVICE_PATH="/dev/input/js0"

echo "***** Setting Up Controller (PS3) *****"
echo "Auto-repeate Rate: "$REFRESH_RATE

rosparam set joy_node/dev $DEVICE_PATH
rosparam set joy_node/deadzone $DEAD_ZONE # The amount the joysitck travels until off-center
rosparam set joy_node/autorepeat_rate $REFRESH_RATE # autorepeat publishing even when input unchanging (Hz)

# run joy_node
rosrun joy joy_node