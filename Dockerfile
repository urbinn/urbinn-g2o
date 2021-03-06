FROM ubuntu:17.10

WORKDIR /urbinn-g2o
ADD . /urbinn-g2o

RUN apt-get update && apt-get install -y --no-install-recommends \
	build-essential \
	cmake \
	libeigen3-dev \
	python3.6-dev \
	libsuitesparse-dev \
	python3-setuptools \
	python3-pip \
	libopencv-dev \
	python-opencv \
	&& rm -rf /var/lib/apt/lists/* \
	&& /urbinn-g2o/scripts/bootstrap.sh \
	&& ldconfig \
	&& pip3 install \
		pybind11 \
		numpy \
		pytest \