# This is a comment
FROM ubuntu:14.04
MAINTAINER Aaron Marburg <amarburg@apl.washington.edu>

RUN  apt-get update && \
     apt-get install -y cmake rake \
		libopencv-dev libboost-all-dev libeigen3-dev \
		libtclap-dev libgomp1 libsuitesparse-dev git \
		libglew-dev libglm-dev autoconf libtool freeglut3-dev \
    libxmu-dev libxi-dev

RUN groupadd -r lsdslam && useradd -r -g lsdslam lsdslam
USER lsdslam
WORKDIR /opt/lsd_slam

ENTRYPOINT ["rake"]
ENV LSDSLAM_BUILD_DIR build_docker
