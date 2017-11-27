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
	&& rm -rf /var/lib/apt/lists/* \
	&& /urbinn-g2o/bootstrap.sh \
	&& ldconfig \
	&& pip3 install pybind11
#CMD python3 /urbinn-g2o/g2o.py
