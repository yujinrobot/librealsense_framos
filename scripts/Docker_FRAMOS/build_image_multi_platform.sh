#! /bin/sh

# This script builds docker image of the latest FRAMOS librealsense release
#Version of the FRAMOS librealsense release, should be changed according to the latest available FRAMOS version
LIBRS_VERSION=2.50.13

echo "Building images for librealsense version ${LIBRS_VERSION}"
# Build x86_64 image
docker buildx \
    build \
    --platform linux/amd64 \
    --target librealsense_framos \
    --build-arg LIBRS_VERSION=$LIBRS_VERSION \
    --tag librealsense_framos \
    -o type=docker,dest=- . > librealsense-amd64.tar

# Build arm64 image (slow!)
docker buildx \
    build \
    --platform linux/arm64 \
    --target librealsense_framos \
    --build-arg LIBRS_VERSION=$LIBRS_VERSION \
    --tag librealsense_framos \
    -o type=docker,dest=- . > librealsense-arm64.tar

