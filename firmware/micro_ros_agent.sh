docker run -it --rm \
  --init \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  --privileged \
  --net=host \
  micro-ros-agent-ferrari:jazzy \
  serial --dev /dev/ttyACM0 \
  -b 1000000 -v4