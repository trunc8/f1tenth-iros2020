#!/bin/bash
docker run --env ROS_MASTER_URI=http://localhost:11311 --env ROS_HOSTNAME=localhost -it --name=bebot_container --rm --net=host bebot
