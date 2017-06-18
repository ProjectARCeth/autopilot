#!/bin/sh
#Input arguments: path_name. 
FILE_PATH="/home/arcsystem/paths"
cd "$FILE_PATH"
if [ ! -f "$FILE_PATH/$1/1.bin" ]; then
	echo "Path does not exist !"
	exit 
fi
#Iterate through existing maps.
for iter in $(seq 1 20); do
	if [ ! -f "$FILE_PATH/$1/$iter.bin" ]; then
		break
	fi
	echo "Loading new data, proceeding in a few seconds"
	sleep 0.5
	if ["$iter" = 1 ]; then
		button_start=true
	else
		button_start=false
	fi
	roslaunch arc_launch arc.launch Name:="$FILE_PATH/$1/$iter" Button_Start:="$button_start"
done
#Finish run. 
echo "Thank you for traveling with Autonomous Racing Car !"