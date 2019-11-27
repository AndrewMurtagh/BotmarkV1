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
apt-get -y install automake bison swig

if [ ! -d $LIB_DIRECTORY/sphinxbase ]; then
  echo "sphinxbase not downloaded"
  git clone https://github.com/cmusphinx/sphinxbase.git
fi
cd sphinxbase
./autogen.sh
make
make install
