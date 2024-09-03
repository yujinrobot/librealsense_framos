#! /bin/sh

# By using --device-cgroup-rule flag we grant the docker continer permissions -
# to the camera and usb endpoints of the machine.
# It also mounts the /dev directory of the host platform on the contianer 

# /usr/lib/framos and /etc are shared from host to enable the GigEVisionDriver
docker run -it --rm \
    -v /dev:/dev \
    -v /usr/lib/framos:/usr/lib/framos:rw \
    -v /etc:/etc:rw \
    --network="switchnet" \
    --device-cgroup-rule "c 81:* rmw" \
    --device-cgroup-rule "c 189:* rmw" \
    librealsense_framos

