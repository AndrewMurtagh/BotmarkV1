#!/bin/bash


# *****************************************************************
#
#  This file is part of the Botmark benchmark.
#
#  Copyright (c) 2019 Andrew Murtagh, Patrick Lynch,
#  and Conor McGinn.
#
#  This work is licensed under the "Creative Commons
#  (Attribution-NonCommercial-ShareAlike 4.0 International)
#  License" and is copyrighted by Andrew Murtagh, Patrich Lynch,
#  and Conor McGinn.
#
#  To view a copy of this license, visit
#  http://creativecommons.org/licenses/by-nc-sa/4.0/ or
#  send a letter to Creative Commons, PO Box 1866,
#  Mountain View, CA 94042, USA.
#
#  Botmark is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied
#  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
# *****************************************************************/

cd $LIB_DIRECTORY

apt-get install libprotobuf-dev
apt-get install liblmdb-dev
apt-get install libhdf5-serial-dev
apt-get install libopencv-dev
apt-get install libsnappy-dev
apt-get install libleveldb-dev
apt-get install protobuf-compiler

if [ ! -d $LIB_DIRECTORY/caffe ]; then
  echo "caffe not downloaded"
  git clone https://github.com/BVLC/caffe.git
fi
cd caffe
cp Makefile.config.example Makefile.config
sed -i '/^# CPU_ONLY := 1/s/^# //' Makefile.config
mkdir build
cd build
cmake ..
make all
make install
make runtest
cd ../src
protoc caffe/proto/caffe.proto --cpp_out=../include/
