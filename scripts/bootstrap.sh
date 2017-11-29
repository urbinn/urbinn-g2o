#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_ON_SERVER=false
FORCE_BUILD=false

# G2O directory
BA="${DIR}/libs/g2o"

build_g2o() {
  mkdir "${BA}/build" && cd "${BA}/build"
  cmake -B${BA}/build -H${BA} -DG2O_BUILD_EXAMPLES:BOOL=OFF -DG2O_BUILD_APPS:BOOL=OFF
  make -j4
  make install
}

build_bindings() {
  rm -r build
  mkdir build
  cd build
  if [ $BUILD_ON_SERVER = "true" ]; then
    echo $BUILD_ON_SERVER
    cmake -DCSPARSE_INCLUDE_DIR:STRING="/opt/jupyterhub/anaconda/include" ..
  else 
    cmake ..
  fi

  make
}

while test $# -gt 0
do
  case "$1" in
    --server) BUILD_ON_SERVER=true
        ;;
    --*) echo "bad option $1"
        ;;
    *) echo "argument $1"
        ;;
  esac
  shift
done

# Build g2o
if [ ! -d "${BA}/build" ]; then
  build_g2o
fi

# Install pybind nump_cv bindings
#cd ${DIR}/thirdparty/pybind11_opencv_numpy
#python3.6 setup.py build
#python3.6 setup.py install

# Build python bindings
cd ${DIR}
#build_bindings
