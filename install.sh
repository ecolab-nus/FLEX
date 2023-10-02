#!/bin/sh

echo 'Installing Blockwise Morphor Mapper toolchain'

FLEX_ROOT=$PWD
MAPPER_ROOT=$FLEX_ROOT/Blockwise_Morpher_Mapper

cd $MAPPER_ROOT
rm -rf build*

mkdir build
cd build
cmake ../
make
