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
apt-get --yes -y install mpi-default-dev openmpi-bin openmpi-common
apt-get --yes -y install libflann-dev
apt-get --yes -y install libeigen3-dev
apt-get --yes -y install libboost-all-dev
apt-get --yes -y install libvtk5.10 libvtk5-dev libproj-dev
apt-get --yes -y install libqhull*
apt-get --yes -y install libusb-dev
apt-get --yes -y install libgtest-dev
apt-get --yes -y install git-core freeglut3-dev pkg-config
apt-get --yes -y install build-essential libxmu-dev libxi-dev
apt-get --yes -y install libusb-1-0-dev graphviz mono-complete
apt-get --yes -y install qt-sdk openjdk-7-jdk openjdk-7-jre
apt-get --yes -y install phonon-backend-gstreamer
apt-get --yes -y install phonon-backend-vlc



if [ ! -d pcl-pcl-1.9.1 ]; then
  wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz
  tar -xf pcl-1.9.1.tar.gz
  rm pcl-1.9.1.tar.gz
fi
cd pcl-pcl-1.9.1
mkdir build
cd build
cmake ..
make
make install
