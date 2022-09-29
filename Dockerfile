# Copyright (C) 2022, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


FROM ros:foxy-ros-base

# Disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

################ EDGE IMPULSE ##################################

RUN apt update && \
    apt-get install -y libatlas-base-dev libportaudio2 libportaudiocpp0 portaudio19-dev python3-pip && \
    pip3 install pyaudio scikit-build && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy
RUN pip3 install edge_impulse_linux 

################ NANOSAUR EDGE IMPULSE PKGS ####################

# Download and build nanosaur_ei
ENV ROS_WS /opt/ros_ws

# Copy wstool ei.rosinstall
COPY nanosaur_ei/rosinstall/ei.rosinstall ei.rosinstall

RUN mkdir -p ${ROS_WS}/src && \
    vcs import ${ROS_WS}/src < ei.rosinstall && \
    apt update && \
    rosdep install --from-paths $ROS_WS/src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

ADD . $ROS_WS/src/nanosaur_ei

# Change workdir
WORKDIR $ROS_WS

# Build nanosaur edge impulse package
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install --merge-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

######### Download Edge Impulse model #############

ENV EI_MODEL /opt/ei

RUN mkdir -p ${EI_MODEL} && \
    git clone https://github.com/gbr1/edgeimpulse_example_models.git ${EI_MODEL} && \
    chmod +x ${EI_MODEL}/models/ssd/person_detection/person-detection-linux-aarch64-v42.eim
#    chmod +x ${EI_MODEL}/models/models/fomo/beer_vs_can/beer-vs.-cans-fomo-linux-aarch64-v12.eim


########## Setup Model ############################

# https://docs.docker.com/engine/reference/builder/#stopsignal
# https://hynek.me/articles/docker-signals/
STOPSIGNAL SIGINT
# run ros package launch file
CMD ["ros2", "launch", "nanosaur_ei", "edge_impulse.launch.py"]