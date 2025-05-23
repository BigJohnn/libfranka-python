#!/bin/bash
#

if [ ! -d "build" ]; then
    mkdir build
fi

cd build
cmake ..
make -j8
cd ..
pip install -e .
