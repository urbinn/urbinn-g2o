FROM ubuntu:17.10

WORKDIR /urbinn-g2o
ADD . /urbinn-g2o

RUN apt-get update && rm -rf /var/lib/apt/lists/ && apt-get install -y --no-install-recommends \
	cmake \
	libeigen3-dev \
	python3.6 \
	libboost-python-dev \
	libsuitesparse-dev \
	&& rm -rf /var/lib/apt/lists/* \
	&& /urbinn-g2o/bootstrap.sh && ldconfig

CMD python3 /urbinn-g2o/g2o.py
