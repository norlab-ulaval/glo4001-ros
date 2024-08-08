ARG TAG=foxy-desktop
FROM docker.io/osrf/ros:${TAG}

# Update and upgrade
RUN apt update && apt upgrade -y

# Dependencies
RUN apt update && apt install -y ros-foxy-xacro ros-dev-tools ros-foxy-tf-transformations ros-foxy-rosbridge-server ros-foxy-depthai-ros

# Clean up
RUN apt clean && \
    rm -rf /var/lib/apt/lists/*

# Setup .bashrc
RUN echo "alias colcon_make='colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release && source install/setup.bash'" >> ~/.bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

RUN echo "if [ -f ~/rc_car/install/setup.bash ]; then source ~/rc_car/install/setup.bash; fi" >> ~/.bashrc

# ARG REMOTE_USER
# ARG REMOTE_UID
# ARG REMOTE_GID
# RUN addgroup --gid ${REMOTE_GID} ${REMOTE_USER} && adduser --disabled-password --uid ${REMOTE_UID} --gid ${REMOTE_GID} ${REMOTE_USER}

# ENV HOME /home/${REMOTE_USER}

# USER ${REMOTE_USER}
