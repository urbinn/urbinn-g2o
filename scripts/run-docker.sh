#!/bin/bash
DIRECTORY=$1

if [ -d "$1" ]; then
  docker build -t urbinn/g2o -f $1/Dockerfile .
  # Base a development image on the base image
  docker run --name urbinn-g2o-dev -v $DIRECTORY:/urbinn-g2o -it urbinn/g2o /bin/bash
else
  echo $1
  echo "Excepting second argument: host project directory."
fi
