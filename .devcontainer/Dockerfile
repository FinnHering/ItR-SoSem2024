FROM ros:latest
LABEL authors="Finn Hering <finn@hering.sh>"

ARG ros_dist="humble"
ARG work_dir="/usr/src/ITR"
ARG gazebo_version="harmonic"
ENV ROS_DOMAIN_ID=69
ENV TURTLEBOT3_MODEL=burger
#source /usr/share/gazebo/setup.sh
ENV GAZEBO_MODEL_PATH=$$GAZEBO_MODEL_PATH:$work_dir/driving_swarm_infrastructure/src/driving_swarm_bringup/models/:/opt/ros/humble/share/turtlebot3_gazebo/models
ENV GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$work_dir/driving_swarm_infrastructure/src/driving_swarm_bringup/worlds/:/opt/ros/humble/share/turtlebot3_gazebo/models
ENV PYTHONWARNINGS=ignore:::setuptools.command.install

# Setup gazebo



RUN apt-get update

# Install prerequisites for installing gezebo
RUN apt-get install -y lsb-release wget gnupg

# Install gezebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update

# Don't install gazebo manually! The ros metapackage will pick the right version
# RUN apt-get install -y "gz-$gazebo_version"

# gazebo will be installed here as an dependency:
RUN apt-get install -y "ros-$ros_dist-gazebo-ros-pkgs"


# Install QT
RUN apt-get install -y qt6-base-dev

# Install ROS(2)
RUN apt-get install -y "ros-$ros_dist-desktop"
RUN apt-get install -y "ros-$ros_dist-rqt*"

RUN apt-get install -y "python-pip"
RUN apt-get install -y "python3-pip"

# Update default bashrc to automatically source the ros environment
RUN echo "source /opt/ros/$ros_dist/setup.bash" >> /etc/bash.bashrc
RUN echo "source /opt/ros/$ros_dist/local_setup.bash" >> /etc/bash.bashrc
RUN echo "source $work_dir/driving_swarm_infrastructure/install/local_setup.bash" >> /etc/bash.bashrc
RUN echo "source $work_dir/introduction_to_robotics_tutorial/install/local_setup.bash" >> /etc/bash.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /etc/bash.bashrc


WORKDIR /usr/src/ITR
