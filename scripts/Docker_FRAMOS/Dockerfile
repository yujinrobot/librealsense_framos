ARG BASE_IMAGE=ubuntu:20.04 
#################################
#   Librealsense Builder Stage  #
#################################

FROM $BASE_IMAGE as librealsense_framos-builder
# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive
# Builder dependencies installation
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \    
    curl \
    python3 \
    python3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*


# Download sources from FRAMOS site
WORKDIR /usr/src
RUN curl https://www.framos.com/framos3d/D400e/Software/Latest/FRAMOS_D400e_Software_Package_Linux64_x64_latest.tar.gz -o librealsense.tar.gz
RUN tar -vxf librealsense.tar.gz \
    && rm librealsense.tar.gz \
    && apt-get update \
    && ln -s /usr/src/librealsense-$LIBRS_VERSION /usr/src/librealsense \
    && apt-get install \
        --no-install-recommends \
        --assume-yes \
        ./FRAMOS_D400e_Software_Package/FRAMOS*.deb \
    && echo ". /etc/profile.d/camerasuite.sh" >> /root/.bashrc

# CameraSuite environment variables have to be passed from host
ENV CAMERA_SUITE_SRC_PATH="/usr/src/framos/camerasuite" 
ENV CAMERA_SUITE_LIB_PATH="/usr/lib/framos/camerasuite" 
ENV CAMERA_SUITE_INCLUDE_PATH="/usr/include/framos/camerasuite" 
ENV CAMERA_SUITE_DOC_PATH="/usr/doc/framos/camerasuite"  
ENV CAMERA_SUITE_PATH=$CAMERA_SUITE_SRC_PATH 
ENV CAMERA_SUITE_PACKAGE="/usr/src/framos/camerasuite/CMakePackages"
ENV CAMERA_SUITE_TOOLCHAIN_PATH="/usr/src/framos/camerasuite/Toolchain" 
ENV CAMERA_SUITE_GENICAM_ROOT_V3_0="/usr/src/framos/camerasuite/GenICam"
ENV GENICAM_LOG_CONFIG_V3_0="/usr/src/framos/camerasuite/GenICam/log/config-unix/DefaultLogging.properties"
ENV CAMERA_SUITE_GENICAM_CACHE_V3_0="$HOME/.cache/genicam"
ENV CAMERA_SUITE_CUSTOM_PATH="$HOME/.local/custom"

# Build and install
RUN cd /usr/src/librealsense2 \
    && mkdir build && cd build \
    && cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \    
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ../ \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=TRUE \
    && make -j$(($(nproc)-1)) all \
    && make install 

######################################
#   librealsense Base Image Stage    #
######################################
FROM ${BASE_IMAGE} as librealsense_framos
ARG LIBRS_VERSION
# Copy binaries from builder stage
COPY --from=librealsense_framos-builder /opt/librealsense /usr
COPY --from=librealsense_framos-builder /usr/src/librealsense2/build/wrappers/python/pyrealsense2.cpython-36m-x86_64-linux-gnu.so.${LIBRS_VERSION}* /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense_framos-builder /usr/src/librealsense2/config/99-realsense-libusb.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib
ENV PATH=$PATH:/usr/local/lib

# Install dep packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \	
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    kmod \
    && rm -rf /var/lib/apt/lists/*

# Shows a list of connected Realsense devices
CMD [ "rs-enumerate-devices", "--compact" ]
