# Reuse the deployed teleop image and its matching libfranka. No hardware startup.
ARG TELEOP_IMAGE=franka-upper-body-teleop:latest
FROM ${TELEOP_IMAGE}
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake libfmt-dev libgl1 libglib2.0-0 \
    python3-venv python3-pip python3-pytest \
    ros-humble-rclpy ros-humble-sensor-msgs ros-humble-cv-bridge \
    ros-humble-tf2-ros ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
# Match ROS Humble's Python 3.10; keep numerical/vision packages isolated from teleop.
# NumPy 1.x is intentional for the packaged cv_bridge ABI.
RUN python3 -m venv --system-site-packages /opt/handeye-venv \
    && /opt/handeye-venv/bin/python -m pip install --no-cache-dir \
       numpy==1.26.4 scipy==1.13.1 opencv-contrib-python==4.10.0.84
ENV HANDEYE_PYTHON=/opt/handeye-venv/bin/python
