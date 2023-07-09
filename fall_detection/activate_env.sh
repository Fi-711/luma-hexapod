#!/bin/bash
activate () {
  . /home/pi-ubuntu/opencv/bin/activate
}
activate
export DISPLAY=:0
python /home/pi-ubuntu/ros2_hexapod/fall_detection/fall_detection.py