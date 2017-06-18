#!/bin/sh
#Input arguments: path_name. 
FILE_PATH="/home/arcsystem/paths"
cd "$FILE_PATH"
mkdir $1
#Launch teach iterations.
for iter in $(seq 1 20); do
	roslaunch arc_launch arc.launch Mode:="false" Name:="$FILE_PATH/$1/$iter" 
	sleep 0.5
	read -p "A next run?[yn] " yn
    case $yn in
        [Yy]* ) continue;;
        [Nn]* ) break;;
        * ) continue;;
    esac
done
#Finish programm.
echo "Run finished, thank you for teaching me !"