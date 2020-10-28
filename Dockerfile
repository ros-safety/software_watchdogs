# Set temporary image for building
FROM ros:foxy as intermediate
# Add metadata identifying these images as build containers
# After build, remove intermediate images with `docker rmi -f $(docker images -q --filter label=stage=intermediate)`
LABEL stage=intermediate

# Clone and build repository
WORKDIR /root
RUN git clone https://github.com/ros-safety/software_watchdogs.git
RUN . /opt/ros/foxy/setup.sh && \
    colcon build

# Choose the base image for the final image
FROM ros:foxy-ros-core

# Install dependencies for demo applications
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-foxy-demo-nodes-cpp \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Copy across the files from our `intermediate` container
RUN mkdir -p /root/install
COPY --from=intermediate /root/install /root/install

# Set up auto-source of workspace
RUN sed --in-place --expression \
      '$isource "/root/install/setup.bash"' \
      /ros_entrypoint.sh
