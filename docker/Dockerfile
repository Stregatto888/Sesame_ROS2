FROM ros:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-rviz2 \
      ros-jazzy-tf2-tools \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-xacro \
      ros-jazzy-moveit \
      ros-jazzy-pick-ik \
      python3-colcon-common-extensions \
      liburdfdom-tools \
      libgl1-mesa-dri \
      mesa-utils \
      x11-utils \
    && rm -rf /var/lib/apt/lists/*

ENV QT_X11_NO_MITSHM=1 \
    QT_QPA_PLATFORM=xcb \
    XDG_RUNTIME_DIR=/tmp/runtime-root

RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root \
 && echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo '[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /root/ros2_ws
CMD ["bash"]
