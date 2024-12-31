#!/bin/bash

xhost +
docker exec -it robotiq_2f85_container bash -it -c "roslaunch robotiq_2f_gripper_control test_85mm_gripper.launch comport:="/dev/ttyUSB0""
sleep 1.