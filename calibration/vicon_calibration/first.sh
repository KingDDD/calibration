#!/bin/bash

# Get Model Number
while [[ -z "$model" ]]
do
	read -p "Enter Model (ex. PEQ_00X): " model
done
echo Model = $model
export model=$model


# Create calibration directory in /var/tmp
dirname="/home/local/beltpack_calibrations/$(date +%Y)-$(date +%m)-$(date +%d)_"$model
dirname_snaps=$dirname"_snaps"
if [ ! -d "$dirname" ]; then
	mkdir -p $dirname
	echo "creating directory $dirname"
else
	while [[ $ans1 != "y" && $ans1 != "n" ]]
	do
		echo "Warning: $dirname already exists!"
		read -p "Delete existing data? (y/n): " ans1
	done
	
	if [ $ans1 = "y" ]; then
		rm -rf $dirname
		mkdir $dirname
		echo "creating directory $dirname"
	else
		echo "exiting script"
		exit 1
	fi
fi

source /home/local/Development/repo_nova/leapcore/malleus/build/set_environment.sh

mldb root
python /home/local/Development/repo_nova/leapcore/malleus/build/bin/collect_wearable_prop_data.py --powerpack 1 --cache $dirname --batchsize 1 --mode vicon_chessboard_data
