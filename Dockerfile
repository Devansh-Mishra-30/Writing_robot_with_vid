# Use an official Ubuntu 20.04 LTS as a parent image
FROM ubuntu:24.04

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get upgrade -y  && \
    apt-get install -y \
    curl \
    python3-pip \
	build-essential \
	libglew2.2 \
	libpython3.12-dev \
	nano \
	python3-numpy \
	python3-yaml \
	python3-tk \
	&& rm -rf /var/lib/apt/lists/*


# Install python packages (WARNING: Only use break-system-packages in container!!!!)
RUN pip install --break-system-packages \
    pin\
    pybullet\
	matplotlib\
	scipy

WORKDIR /workspace/

# Set the default command to execute
# When creating a container, this will simulate `docker run -it`
CMD ["bash"]

