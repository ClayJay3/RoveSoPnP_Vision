# Image Variables
ARG UBUNTU_MAJOR="24"

# Base Image
FROM ubuntu:${UBUNTU_MAJOR}.04

# Install Variables
ARG UBUNTU_MAJOR="24"

# Set Non-Interactive Mode
ARG DEBIAN_FRONTEND=noninteractive

# Set Timezone
ARG TZ="America/Chicago"
RUN echo "${TZ}" > /etc/localtime && \
    echo "${TZ}" > /etc/timezone

# Set the default shell to bash with pipefail option. This ensures that the shell exits immediately if any command exits with a non-zero status.
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
    # Core Build Tools
    build-essential \
    cmake \
    git \
    pkg-config \
    gcc-12 \ 
    g++-12 \
    # Python Build Headers
    python3-dev \
    python3-pip \
    python3-numpy \
    python-is-python3 \
    # Image I/O (Required for OpenCV imread/imwrite)
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    # Video I/O & Codecs (Required for OpenCV VideoCapture)
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libx265-dev \
    # GUI Support (Required for OpenCV imshow/highgui)
    libgtk-3-dev \
    libgl1-mesa-dev \
    # Math & Parallelization Optimizations
    libatlas-base-dev \
    libtbb-dev \
    # System Utilities (Required for fetching sources & hardware access)
    ca-certificates \
    curl \
    wget \
    udev \
    usbutils \
    fish \
    bash-completion \
    nlohmann-json3-dev \
    libeigen3-dev \
    tzdata \
    ssh && \
    rm -rf /var/lib/apt/lists/*

# Set Timezone
RUN echo "${TZ}" > /etc/localtime && \
    echo "${TZ}" > /etc/timezone

# Install CMake
ARG CMAKE_VERSION="3.30.2"
COPY install-cmake.sh /tmp/
RUN if [ "${CMAKE_VERSION}" != "none" ]; then \
    chmod +x /tmp/install-cmake.sh && /tmp/install-cmake.sh ${CMAKE_VERSION}; \
    fi && \
    rm -f /tmp/install-cmake.sh

# Set Working Directory
WORKDIR /opt

# Install Quill
ARG QUILL_VERSION="9.0.2"
RUN git clone --depth 1 --branch "v${QUILL_VERSION}" https://github.com/odygrd/quill.git && \
    cmake -S quill -B quill/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DQUILL_FMT_EXTERNAL=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build quill/build --target install -j$(nproc) && \
    rm -rf quill

# Install OpenCV.
ARG OPENCV_VERSION="4.10.0" 
RUN git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git && \
    git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git && \
    mkdir opencv/build && cd opencv/build && \
    cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_SHARED_LIBS=OFF \
    -D WITH_CSTRIPES=ON \
    -D WITH_OPENCL=ON \
    -DBUILD_opencv_wechat_qrcode=OFF \
    -D BUILD_opencv_tracking=ON \
    -D ENABLE_FAST_MATH=1 \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules .. && \
    make install -j8 && ldconfig && \
    cd ../.. && \
    rm -rf opencv_contrib && \
    rm -rf opencv

# Set Fish as Default Shell.
RUN chsh -s /usr/bin/fish

# Clone RoveSoPNP Repository
RUN git clone --recurse-submodules -j8 https://github.com/ClayJay3/RoveSoPNP_Vision.git

# Set Working Directory
WORKDIR /opt/RoveSoPNP/

# Set Labels
LABEL authors="Missouri S&T RoveSoPNP Design Team"
LABEL maintainer="claytonraycowen@gmail.com"
LABEL org.opencontainers.image.source=https://github.com/ClayJay3/RoveSoPNP_Vision.git
LABEL org.opencontainers.image.licenses=GPL-3.0-only
LABEL org.opencontainers.image.version="v24.5.0"
LABEL org.opencontainers.image.description="Docker Image for Ubuntu ${UBUNTU_MAJOR} with OpenCV ${OPENCV_VERSION}, Quill ${QUILL_VERSION}."
