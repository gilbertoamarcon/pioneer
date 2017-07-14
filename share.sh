#!/bin/bash
## Share

# I use this script to share the package with other computers - and robots
#

# Usage: ./share.sh aadi2@aadi2
# 

dest=$1
dir=$(pwd)
com="scp -r $dir $dest:ros-ws/mobile/src"
echo $com
$com
