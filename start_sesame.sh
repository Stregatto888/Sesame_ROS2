#!/usr/bin/env bash
# Avvia Docker, abilita X11, entra nel container e lancia la visualizzazione di Sesame.
set -e

systemctl is-active --quiet docker || sudo systemctl start docker
xhost +local:docker

if [ "$(docker inspect -f '{{.State.Running}}' sesame_gpu)" != "true" ]; then
  docker start sesame_gpu > /dev/null
fi

docker exec -it sesame_gpu bash -ic 'ros2 launch sesame_description display.launch.py'
