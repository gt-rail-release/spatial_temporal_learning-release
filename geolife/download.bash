#!/bin/bash

# GeoLife Data Set Download Script
#
# Author: Russell Toris - rctoris@wpi.edu

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Give the user the license first
cat $DIR/LICENSE
echo
echo "This script will download and install the Microsoft Research Asia GPS Trajectories."
echo

# Confirmation prompt
while true; do
	read -p "Do you agree to the above terms and conditions? [Y/n] " yn
	case $yn in
		[Yy]* ) break;;
		[Nn]* ) exit;;
		* ) echo "Please answer with 'y' or 'n'";;
	esac
done

# Download and unzip the data
mkdir -p /tmp/geolife
cd /tmp/geolife
wget http://ftp.research.microsoft.com/downloads/b16d359d-d164-469e-9fd4-daa38f2b2e13/Geolife%20Trajectories%201.3.zip
echo "Uncompressing files..."
unzip *.zip >> /dev/null

# Check if we need sudo
echo "Copying files to '$DIR'..."
if [[ -w $DIR ]]
then
	cp Geolife\ Trajectories\ 1.3/Data/000/Trajectory/*.plt $DIR
else
	sudo cp Geolife\ Trajectories\ 1.3/Data/000/Trajectory/*.plt $DIR
fi

rm -rf /tmp/geolife
