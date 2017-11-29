#!/bin/bash
PRODUCTION=false
DEVELOPMENT=false
DIRECTORY=$2

while test $# -gt 0
do
  case "$1" in
    --prod) PRODUCTION=true
        ;;
    --dev) DEVELOPMENT=true
        ;;
    --*) echo "bad option $1"
        ;;
    *) echo "argument $1"
        ;;
  esac
  shift
done

if [ $PRODUCTION = "true" ]; then
  docker build -t urbinn/g2o -f ./.docker/dockerfile .
elif [ $DEVELOPMENT = "true" ]; then
  if [ $2 ]; then
    docker build -t urbinn/g2o-dev -f ./.docker/dev.dockerfile .
    docker run --name urbinn-g2o-dev -v $DIRECTORY:/urbinn-g2o -it urbinn/g2o-dev /bin/bash
  else
    echo "Excepting second argument: host project directory."
  fi
fi