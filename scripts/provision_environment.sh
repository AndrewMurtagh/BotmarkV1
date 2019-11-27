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
export LIB_DIRECTORY


apt-get -y install libboost-all-dev festival-dev festival

./installs/mrpt.sh

./installs/caffe.sh

./installs/sphinxbase

./installs/pocketsphinx

./installs/opencv

./installs/pcl

./installs/gmapping


cd $LIB_DIRECTORY/../scripts
