FROM ros:humble-ros-base-jammy AS ateam_workspace

ARG HOST_SRC=./
ENV WORKSPACE=/workspace

RUN mkdir -p ${WORKSPACE}

COPY ${HOST_SRC} ${WORKSPACE}/src

WORKDIR ${WORKSPACE}

# Deps
# probably dont need the first two lines like source and update seperate from rodep update but I cant remember
RUN apt-get update \
    && . /opt/ros/humble/setup.sh \
    && rosdep update \
    && rosdep install --from-paths . --ignore-src -y \
    && ./src/ateam_ui/install_deps.sh

# should probably split ui and base into two images and also provide a runtime version without the source code dir
# RUN ./src/software/ateam_ui/install_deps.sh

RUN . /opt/ros/humble/setup.sh && colcon build

# replace entrypoint file with effects of bash rc and change default entrypoint
# I just want to kill myself I forgot the most basic things about docker like that it runs /bin/sh by default in a non interactive, non login shell
RUN cp ${WORKSPACE}/src/ateam_bringup/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Will fail because of docker files
CMD ["ros2", "launch", "ateam_bringup", "bringup_simulation.launch.py"]


FROM ateam_workspace AS ssl_log_replay
WORKDIR ${WORKSPACE}
RUN ./src/ateam_bringup/scripts/install_ssl_logging.sh