FROM ubuntu:17.10

WORKDIR /urbinn-g2o
ADD . /urbinn-g2o

RUN apt-get update && apt-get install -y \
	cmake \
	libsuitesparse-dev \
	libeigen3-dev \
	python3.6 \
	libboost-python-dev \
	libsuitesparse-dev \
	&& rm -rf /var/lib/apt/lists/ \
	&& /urbinn-g2o/bootstrap.sh