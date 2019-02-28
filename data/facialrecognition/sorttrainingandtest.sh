#!/bin/bash

workingDir=$1
personNum=0

mkdir $workingDir"/../testing"
mkdir $workingDir"/../training"

for i in $(ls $workingDir)
do
	index=0
	total=$(ls -1 $(echo $workingDir"/"$i) | wc -l)
	training="$(echo "( $total * 0.8 )/1" | bc)"
	echo training: $training
	testing=$((  $total - $training ))
	echo testing: $testing
	for file in $(ls -1 $workingDir"/"$i)
	do
		if [ $index -lt $training ]
		then
			echo Moving $file to training
			echo $file";"$personNum >> training.csv
			mv $workingDir"/"$i"/"$file $workingDir"/../training/"$file
		else
			echo Moving $file to testing
			echo $file";"$personNum >> testing.csv
			mv $workingDir"/"$i"/"$file $workingDir"/../testing/"$file
		fi
		index=$(( index + 1 ))
	done
	personNum=$(( personNum + 1))
done
