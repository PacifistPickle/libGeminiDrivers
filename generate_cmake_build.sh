#!/bin/bash

# Find directory this script is located in
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

# Create build directory for each architecure if necessary
mkdir -p $DIR/build/x86/
mkdir -p $DIR/build/RasPi/

# Generate CMake for x86 Build
pushd $DIR/build/x86/
cmake ../../
make clean
make -j4
popd

# Generate CMake for RasPi Build
pushd $DIR/build/RasPi/
cmake -DCMAKE_TOOLCHAIN_FILE:PATH=./toolchain/raspi_toolchain.txt ../../
make clean
make -j4
popd
