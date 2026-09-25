#!/usr/bin/env bash

export __ISAACLAB_TMP_DIR=/tmp/tmp.SJRxCw1v0z
export __ISAACLAB_TMP_XAUTH=$__ISAACLAB_TMP_DIR/tmp.oqamFh8Cpg.xauth

DOCKER_NAME_SUFFIX=-tung docker compose -p tung --file docker-compose.yaml --file x11.yaml --env-file .env.base --env-file .env.ros2 up -d isaac-lab-ros2
# DOCKER_NAME_SUFFIX=-tung docker compose --file docker-compose.yaml --file x11.yaml --env-file .env.base --env-file .env.ros2 up -d isaac-lab-ros2