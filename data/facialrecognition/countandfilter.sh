#!/bin/bash

workingDir=$1
Keeping=0
Removing=0
for i in $(ls $workingDir)
do
	if [ "$(ls -1 $(echo $workingDir"/"$i) | wc -l)" -lt "16" ]
	then
	#	echo "Removing $i"
		Removing=$(($Removing + 1))
		mv $(echo $workingDir"/"$i) temp
	else
		Keeping=$(($Keeping + 1))
	fi

done

echo "Kept $Keeping faces"
echo "Removing $Removing faces"
