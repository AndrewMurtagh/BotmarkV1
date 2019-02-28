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
# * License" and is copyrighted by Andrew Murtagh, Patrich Lynch,
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

FILENAME='../res/system_details.txt'
touch $FILENAME
echo -e "System Details for Botmark Benchmark\n\n" > $FILENAME

echo -e "Architecture:\n $(uname -p)\n\n" >> $FILENAME

echo -e "Kernel Version:\n $(uname -r)\n\n" >> $FILENAME

echo -e "Compiler Version:\n $(g++ --version)\n\n" >> $FILENAME

echo -e "Linker Version:\n $(ld --version)\n\n" >> $FILENAME

echo -e "CPU details:\n $(lscpu)\n\n" >> $FILENAME

echo -e "More CPU details:\n $(cat /proc/cpuinfo)\n\n" >> $FILENAME

echo -e "Disk details:\n $(df -h)\n\n" >> $FILENAME

echo -e "Memory details:\n $(cat /proc/meminfo)\n\n" >> $FILENAME

echo -e "More system details:\n $(lshw)\n\n" >> $FILENAME

#dmidecode -t 17 | awk '( /Size/ && $2 ~ /^[0-9]+$/ ) { x+=$2 } END{ print "\t" "Installed Ram: " x "MB"}'
