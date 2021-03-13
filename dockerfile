FROM ros:kinetic-ros-base

ARG DOCKER_USER=doogie

RUN useradd -s /bin/bash ${DOCKER_USER}

# install gosu
RUN set -eux; \
    apt-get update; \
    apt-get install -y gosu; \
    rm -rf /var/lib/apt/lists/*; \
    gosu nobody true

# install common dev tools
RUN export DEBIAN_FRONTEND=noninteractive; \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    bash-completion \
    pkg-config \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /home/${DOCKER_USER} && chown ${DOCKER_USER}:${DOCKER_USER} /home/${DOCKER_USER};

WORKDIR /home/${DOCKER_USER}

COPY docker-entrypoint.sh /usr/bin/docker-entrypoint.sh

ENTRYPOINT ["docker-entrypoint.sh"]

CMD ["bash"]