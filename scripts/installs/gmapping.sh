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
#apt-get --yes -y install subversion
#svn checkout http://svn.openslam.org/data/svn/gmapping/
git clone https://github.com/OpenSLAM-org/openslam_gmapping.git

cd openslam_gmapping
sed -i -e 's/.*SUBDIRS=utils  sensor log configfile scanmatcher carmenwrapper gridfastslam gui gfs-carmen.*/SUBDIRS=utils  sensor log configfile scanmatcher carmenwrapper gridfastslam gfs-carmen/' Makefile
sed -i -e 's/.*SUBDIRS=utils sensor log configfile scanmatcher gridfastslam gui.*/SUBDIRS=utils sensor log configfile scanmatcher gridfastslam/' Makefile
sed -i '147s/for (unsigned int i=0; i< dim; i++){/for (unsigned int j=0; j< dim; j++){/' gridfastslam/gfs2rec.cpp
sed -i '10i#define MAXDOUBLE 1e1000' particlefilter/particlefilter.h
./configure
make

cp -r lib/* /usr/local/lib/
mkdir /usr/local/include/gmapping
mkdir /usr/local/include/gmapping/gridfastslam

cp gridfastslam/gridslamprocessor.h /usr/local/include/gmapping/gridfastslam/
mkdir -p /usr/local/include/gmapping/sensor/sensor_base
cp sensor/sensor_base/sensor.h /usr/local/include/gmapping/sensor/sensor_base/
mkdir /usr/local/include/gmapping/particlefilter
cp particlefilter/particlefilter.h /usr/local/include/gmapping/particlefilter/
mkdir /usr/local/include/gmapping/utils
cp utils/gvalues.h /usr/local/include/gmapping/utils/
cp utils/point.h /usr/local/include/gmapping/utils/
cp utils/macro_params.h /usr/local/include/gmapping/utils/
mkdir /usr/local/include/gmapping/log
cp log/sensorlog.h /usr/local/include/gmapping/log/
cp log/carmenconfiguration.h /usr/local/include/gmapping/log/
cp sensor/sensor_base/sensorreading.h /usr/local/include/gmapping/sensor/sensor_base/
mkdir -p /usr/local/include/gmapping/sensor/sensor_odometry/
cp sensor/sensor_odometry/odometrysensor.h /usr/local/include/gmapping/sensor/sensor_odometry/
mkdir -p /usr/local/include/gmapping/sensor/sensor_range
cp sensor/sensor_range/rangesensor.h /usr/local/include/gmapping/sensor/sensor_range/
cp sensor/sensor_odometry/odometryreading.h /usr/local/include/gmapping/sensor/sensor_odometry/
cp sensor/sensor_range/rangereading.h /usr/local/include/gmapping/sensor/sensor_range/
cp log/configuration.h /usr/local/include/gmapping/log
mkdir -p /usr/local/include/gmapping/scanmatcher
cp scanmatcher/scanmatcher.h /usr/local/include/gmapping/scanmatcher/
cp scanmatcher/icp.h /usr/local/include/gmapping/scanmatcher/
cp scanmatcher/smmap.h /usr/local/include/gmapping/scanmatcher/
mkdir -p /usr/local/include/gmapping/grid
cp grid/map.h /usr/local/include/gmapping/grid
cp grid/accessstate.h /usr/local/include/gmapping/grid
cp grid/array2d.h /usr/local/include/gmapping/grid
cp grid/harray2d.h /usr/local/include/gmapping/grid
cp utils/autoptr.h /usr/local/include/gmapping/utils/
cp utils/stat.h /usr/local/include/gmapping/utils/
cp gridfastslam/motionmodel.h /usr/local/include/gmapping/gridfastslam/
cp gridfastslam/gridslamprocessor.hxx /usr/local/include/gmapping/gridfastslam/
cp gridfastslam/gfsreader.h /usr/local/include/gmapping/gridfastslam/
cp utils/commandline.h /usr/local/include/gmapping/utils/
mkdir -p /usr/local/include/gmapping/carmenwrapper
cp carmenwrapper/carmenwrapper.h /usr/local/include/gmapping/carmenwrapper/
cp utils/orientedboundingbox.h /usr/local/include/gmapping/utils/
cp utils/orientedboundingbox.hxx /usr/local/include/gmapping/utils/
mkdir -p /usr/local/include/gmapping/configfile/
cp configfile/configfile.h /usr/local/include/gmapping/configfile/
cp log/sensorstream.h /usr/local/include/gmapping/log/
