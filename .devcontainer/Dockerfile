FROM osrf/ros:noetic-desktop-full

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# # Add sudo support for the non-root user
# RUN apt-get update && \
#     apt-get install -y sudo && \
#     echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
#     chmod 0440 /etc/sudoers.d/$USERNAME

# # Switch from root to user
# USER $USERNAME

# Add user to video group to allow access to webcam
# RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install some useful tools
RUN sudo apt install -y \
        git \
        software-properties-common  \
        python3-osrf-pycommon \
        python3-catkin-tools \
        libcanberra-gtk* \
        build-essential \
        python3-rosdep \
        python3-serial\
        python3-pymodbus \ 
        python3-arrow \
        vim \
        && sudo rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws/src && \
/bin/bash -c "cd /catkin_ws/src && source /opt/ros/noetic/setup.bash && catkin_init_workspace" && \
/bin/bash -c "cd /catkin_ws && source /opt/ros/noetic/setup.bash && catkin build" && \
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc && \
echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/catkin_ws" >> ~/.bashrc && \
echo "export ROS_WORKSPACE=/catkin_ws" >> ~/.bashrc
    
WORKDIR /catkin_ws/src/

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################
