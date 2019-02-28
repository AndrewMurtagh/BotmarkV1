#!/bin/bash


# *****************************************************************
# *
# * This file is part of the Botmark benchmark.
# *
# * Copyright (c) 2019 Andrew Murtagh, Patrick Lynch,
# * and Conor McGinn.
# *
# * This work is licensed under the "Creative Commons
# * (Attribution-NonCommercial-ShareAlike 4.0 International)
# * License" and is copyrighted by Andrew Murtagh, Patrick Lynch,
# * and Conor McGinn.
# *
# * To view a copy of this license, visit
# * http://creativecommons.org/licenses/by-nc-sa/4.0/ or
# * send a letter to Creative Commons, PO Box 1866,
# * Mountain View, CA 94042, USA.
# *
# * Botmark is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied
# * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
# * PURPOSE.
# *
# *****************************************************************/




apt-get update

cd ..
mkdir libs
cd libs
LIB_DIRECTORY=`pwd`


#boost
installBoost() {
	apt-get --yes --force-yes install libboost-all-dev
}


#fcl for openrave
installFCL() {
	cd $LIB_DIRECTORY
	apt-get --yes --force-yes install libccd-dev
	git clone https://github.com/flexible-collision-library/fcl.git
	cd fcl
	git checkout 0.5.0  # use FCL 0.5.0
	mkdir build
	cd build
	cmake ..
	make
	make install
}

#collada for openrave
installCollada() {
	cd $LIB_DIRECTORY
	git clone https://github.com/rdiankov/collada-dom.git
	cd collada-dom
	mkdir build
	cd build
	cmake ..
	make
	make install
}

#osc for openrave
installOpenSceneGraph() {
	apt-get install -y libcairo2-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev
	git clone --branch OpenSceneGraph-3.4 https://github.com/openscenegraph/OpenSceneGraph.git
	cd OpenSceneGraph
	mkdir build
	cd build
	cmake .. -DDESIRED_QT_VERSION=4
	make
	make install
}



#openrave
installOpenRAVE() {
	apt-get install -y libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev
	cd $LIB_DIRECTORY
	git clone --branch latest_stable https://github.com/rdiankov/openrave.git
	cd openrave
	mkdir build
	cd build
	cmake ..
	make
	sudo make install
}






# mrpt
installMRPT() {
	add-apt-repository --yes --force-yes ppa:joseluisblancoc/mrpt-1.5
	apt-get update
	apt-get --yes --force-yes install libmrpt-dev mrpt-apps
}




#caffe
installCaffe() {
	cd $LIB_DIRECTORY
	sudo apt-get --yes --force-yes install libhdf5-serial-dev liblmdb-dev libleveldb-dev libsnappy-dev
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
}



#sphinxbase for pocketshpinx
installSphinxBase() {
	cd $LIB_DIRECTORY
	sudo apt-get --yes --force-yes install automake bison swig

	if [ ! -d $LIB_DIRECTORY/sphinxbase ]; then
		echo "sphinxbase not downloaded"
		git clone https://github.com/cmusphinx/sphinxbase.git
	fi
	cd sphinxbase
	./autogen.sh
	make
	make install
}


#pocketshpinx
installPocketSphinx() {
	apt-get --yes --force-yes install libsndfile1-dev


	cd $LIB_DIRECTORY
	git clone https://github.com/cmusphinx/pocketsphinx.git
	cd pocketsphinx
	./autogen.sh
	make
	make install
}




#opencv
installOpenCV() {
	cd $LIB_DIRECTORY
	apt-get --yes --force-yes install build-essential checkinstall pkg-config yasm
	apt-get --yes --force-yes install gfortran
	apt-get --yes --force-yes install libjpeg8-dev libjasper-dev libpng12-dev
	apt-get --yes --force-yes install libtiff5-dev
	apt-get --yes --force-yes install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
	apt-get --yes --force-yes install libxine2-dev libv4l-dev
	apt-get --yes --force-yes install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
	apt-get --yes --force-yes install qt5-default libgtk2.0-dev libtbb-dev
	apt-get --yes --force-yes install libatlas-base-dev
	apt-get --yes --force-yes install libfaac-dev libmp3lame-dev libtheora-dev
	apt-get --yes --force-yes install libvorbis-dev libxvidcore-dev
	apt-get --yes --force-yes install libopencore-amrnb-dev libopencore-amrwb-dev
	apt-get --yes --force-yes install x264 v4l-utils
	apt-get --yes --force-yes install libprotobuf-dev protobuf-compiler
	apt-get --yes --force-yes install libgoogle-glog-dev libgflags-dev
	apt-get --yes --force-yes install libgphoto2-dev libeigen3-dev libhdf5-dev

	if [ ! -d cv ]; then
		git clone https://github.com/opencv/opencv.git
	fi

	cd opencv
	git checkout 2.4
	cd ..

	if [ ! -d opencv_contrib ]; then
		git clone https://github.com/opencv/opencv_contrib.git
	fi
	cd opencv_contrib
	git checkout 2.4
	cd ..

	cd opencv
	mkdir build
	cd build

	cmake -D CMAKE_BUILD_TYPE=RELEASE \
	      -D CMAKE_INSTALL_PREFIX=/usr/local \
	      -D INSTALL_C_EXAMPLES=ON \
	      -D INSTALL_PYTHON_EXAMPLES=ON \
	      -D WITH_TBB=ON \
	      -D WITH_V4L=ON \
	      -D WITH_QT=ON \
	      -D WITH_OPENGL=ON \
	      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
	      -D BUILD_EXAMPLES=ON ..

	make
	make install
	sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
	ldconfig
}



#festival
installFestival() {
	apt-get --yes --force-yes install festival-dev festival
}



#pcl
installPCL() {
	cd $LIB_DIRECTORY
	sudo apt-get --yes --force-yes install mpi-default-dev openmpi-bin openmpi-common
	sudo apt-get --yes --force-yes install libflann1.8 libflann-dev
	sudo apt-get --yes --force-yes install libeigen3-dev
	sudo apt-get --yes --force-yes install libboost-all-dev
	sudo apt-get --yes --force-yes install libvtk5.8-qt4 libvtk5.8 libvtk5-dev libproj-dev
	sudo apt-get --yes --force-yes install libqhull*
	sudo apt-get --yes --force-yes install libusb-dev
	sudo apt-get --yes --force-yes install libgtest-dev
	sudo apt-get --yes --force-yes install git-core freeglut3-dev pkg-config
	sudo apt-get --yes --force-yes install build-essential libxmu-dev libxi-dev
	sudo apt-get --yes --force-yes install libusb-1-0-dev graphviz mono-complete
	sudo apt-get --yes --force-yes install qt-sdk openjdk-7-jdk openjdk-7-jre
	sudo apt-get --yes --force-yes install phonon-backend-gstreamer
	sudo apt-get --yes --force-yes install phonon-backend-vlc



	if [ ! -d pcl-pcl-1.9.1 ]; then
		wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz
		tar -xf pcl-1.9.1.tar.gz
		rm pcl-1.9.1.tar.gz
	fi
	pwd
	cd pcl-pcl-1.9.1
	mkdir build
	cd build
	pwd
	cmake ..
	make
	sudo make install

}





#carmen - for gmapping
#cd ../libs
#wget 'https://sourceforge.net/projects/carmen/files/latest/download' -O carmen-0.7.4-beta.tar.gz
#tar -zxvf carmen-0.7.4-beta.tar.gz
#rm -f carmen-0.7.4-beta.tar.gz
#cd carmen-0.7.4-beta/src
#sed -i -e 's/.*CFLAGS += -Wall -W -Werror -D_REENTRANT.*/CFLAGS += -Wall -W -D_REENTRANT/' Makefile.conf
#sed -i -e 's/.*CXXFLAGS += -Wall -W -Werror -D_REENTRANT.*/CXXFLAGS += -Wall -W -D_REENTRANT/' Makefile.conf
#./configure --nographics --no-java
#make
#cd ..



#gmapping
installGmapping() {
	cd $LIB_DIRECTORY
	#sudo apt-get --yes --force-yes install subversion
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


}


installBoost
installCollada
installOpenSceneGraph
installFCL
installOpenRAVE
installMRPT
installCaffe
installSphinxBase
installPocketSphinx
installOpenCV
installFestival
installPCL
installGmapping



cd $LIB_DIRECTORY/../scripts
