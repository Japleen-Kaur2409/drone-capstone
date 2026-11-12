# ArduPilot Copter SITL + MAVProxy, built natively for arm64 (Apple Silicon) or amd64.
FROM ubuntu:22.04

ARG COPTER_TAG=Copter-4.7.2
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        git ca-certificates build-essential g++ python3 python3-pip python3-dev \
        libxml2-dev libxslt1-dev pkg-config \
    && rm -rf /var/lib/apt/lists/*

# empy must stay on 3.x for ArduPilot's waf build
RUN pip3 install --no-cache-dir empy==3.3.4 future pexpect ptyprocess \
        pymavlink MAVProxy

WORKDIR /ardupilot
RUN git clone --depth 1 --branch ${COPTER_TAG} --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git . \
    && ./waf configure --board sitl \
    && ./waf copter \
    && rm -rf .git modules/*/.git

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 14550/14551: MAVProxy TCP outputs for a GCS and for our Python scripts
EXPOSE 14550 14551
ENTRYPOINT ["/entrypoint.sh"]
