#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Build g2o
BA="${DIR}/thirdparty/g2o"

build_g2o() {
  mkdir "${BA}/build" && cd "${BA}/build"
  cmake -B${BA}/build -H${BA} -DG2O_BUILD_EXAMPLES:BOOL=OFF
  make
  make install
}

if [ ! -d "${BA}/build" ]; then
  build_g2o
elif [ "$1" = "--force" ]; then
  rm -r build
  build_g2o
else 
  echo "SKIPPING: Build artifact detected in g2o, run with --force to rebuild."
fi

# Build g2o bindings
cd ${DIR}

if [ -d "${DIR}/build" ]; then
  rm -r build
  mkdir build
  cmake ...
  make
else
  mkdir build
  cd build
  cmake ..
  make
fi