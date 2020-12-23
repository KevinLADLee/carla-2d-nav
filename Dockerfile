FROM ubuntu:bionic

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        sudo git vim wget curl build-essential cmake ninja-build clang-8 openssl ca-certificates python3-dev python3-pip rsync zlibc zlib1g-dev  \
    && rm -rf /var/lib/apt/lists/*

RUN ln -s /usr/bin/clang-8 /usr/bin/clang && \
    ln -s /usr/bin/clang++-8 /usr/bin/clang++

RUN useradd -m ubuntu \              
    && usermod -s /bin/bash ubuntu \
    && echo 'ubuntu:ubuntu' | chpasswd \ 
    && chmod u+w /etc/sudoers \
    && echo "ubuntu ALL=(ALL:ALL) ALL" >> /etc/sudoers \
    && chmod u-w /etc/sudoers 

USER ubuntu
WORKDIR /home/ubuntu

ARG CARLA_VERSION=0.9.10.1
RUN git clone https://github.com/carla-simulator/carla.git --progress --depth=1 -b $CARLA_VERSION 

WORKDIR /home/ubuntu/carla
RUN /bin/bash ./Util/BuildTools/Setup.sh
