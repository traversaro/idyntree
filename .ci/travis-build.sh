#!/bin/sh

set -e

mkdir build
cd build
if [ $CODYCO_ICUBWBI_USE_EXTERNAL_TORQUE ]; then
    cmake -G"${TRAVIS_CMAKE_GENERATOR}" -DCODYCO_TRAVIS_CI:BOOL=ON -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} -DNON_INTERACTIVE_BUILD:BOOL=TRUE -DCODYCO_ICUBWBI_USE_EXTERNAL_TORQUE_CONTROL:BOOL=TRUE ..
else
    cmake -G"${TRAVIS_CMAKE_GENERATOR}" -DCODYCO_TRAVIS_CI:BOOL=ON -DCMAKE_BUILD_TYPE=${TRAVIS_BUILD_TYPE} -DNON_INTERACTIVE_BUILD:BOOL=TRUE ..
fi
cmake --build . --config ${TRAVIS_BUILD_TYPE}