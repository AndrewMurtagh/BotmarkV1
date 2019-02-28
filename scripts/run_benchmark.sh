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




echo "Running Person Tracking Benchmark"
../bin/PersonTracking
while [ $? -ne 0 ]; do
	echo -e "\nPerson Tracking Benchmark failed - retrying"
	../bin/PersonTracking
done


echo "Running Facial Recognition Benchmark"
../bin/FacialRecognition
while [ $? -ne 0 ]; do
	echo -e "\nFacial Recognition Benchmark failed - retrying"
	../bin/FacialRecognition
done



echo "Running SLAM Benchmark"
../bin/Slam
while [ $? -ne 0 ]; do
	echo -e "\nSLAM Benchmark failed - retrying"
	../bin/Slam
done



echo "Running Path Planning Benchmark"
../bin/PathPlanning
while [ $? -ne 0 ]; do
	echo -e "\nPath Planning Benchmark failed - retrying"
	../bin/PathPlanning
done



echo "Running Object Recognition Benchmark"
../bin/ObjectRecognition
while [ $? -ne 0 ]; do
	echo -e "\nObject Recognition Benchmark failed - retrying"
	../bin/ObjectRecognition
done


echo "Running Motion Planning Benchmark"
../bin/MotionPlanning
while [ $? -ne 0 ]; do
	echo -e "\nMotion Planning Benchmark failed - retrying"
	../bin/MotionPlanning
done




echo "Running Voice Recognition Benchmark"
../bin/VoiceRecognition
while [ $? -ne 0 ]; do
	echo -e "\nVoice Recognition Benchmark failed - retrying"
	../bin/VoiceRecognition
done



echo "Running Speech Synthesis Benchmark"
../bin/Speechsynthesis
while [ $? -ne 0 ]; do
	echo -e "\nSpeech Synthesis Benchmark failed - retrying"
	../bin/Speechsynthesis
done
