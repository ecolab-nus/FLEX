#!/bin/sh


FLEX_ROOT=$PWD
MAPPER_ROOT=$FLEX_ROOT/Blockwise_Morpher_Mapper
MICROARCHITECTURE_ROOT=$FLEX_ROOT/Microarchitecture


echo '######-----Installing Blockwise Morphor Mapper toolchain------######'

cd $MAPPER_ROOT
rm -rf build*

mkdir build
cd build
cmake ../
make

echo '######-----Synthesis of FLEX RTL------######'

cd $MICROARCHITECTURE_ROOT
rm -rf workdir

source bin/setproj.sh
make gen
make syn UNIT=flex


