#!/usr/bin/env bash

export __ISAACLAB_TMP_DIR=/tmp/tmp.SJRxCw1v0z
mkdir -p $__ISAACLAB_TMP_DIR

export __ISAACLAB_TMP_XAUTH=$__ISAACLAB_TMP_DIR/tmp.oqamFh8Cpg.xauth

touch $__ISAACLAB_TMP_XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $__ISAACLAB_TMP_XAUTH nmerge -
chmod 777 $__ISAACLAB_TMP_XAUTH

docker compose -p able --file docker-compose.yaml --file x11.yaml --env-file .env.base --env-file .env.ros2 up -d isaac-lab-ros2
# DOCKER_NAME_SUFFIX=-tung docker compose --file docker-compose.yaml --file x11.yaml --env-file .env.base --env-file .env.ros2 up -d isaac-lab-ros2