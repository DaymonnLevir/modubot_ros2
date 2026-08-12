FROM ros:humble-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /usr/local/bin/modubot-face-rosbridge
RUN chmod +x /usr/local/bin/modubot-face-rosbridge

ENTRYPOINT ["/usr/local/bin/modubot-face-rosbridge"]
