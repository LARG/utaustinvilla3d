#!/bin/bash
# Copy files needed to run binary to a new directory
# Usage: copy_files_needed_to_run.sh <destination>


if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <destination>"
  exit 1
fi

AGENT_BINARY=agentspark

DIR_DEST="$( cd "$( dirname "$1" )" && pwd )" || exit 1
DEST=$DIR_DEST/$(basename $1)

DIR_SCRIPT="$( cd "$( dirname "$0" )" && pwd )" 
cd $DIR_SCRIPT/..

echo "Copying files needed to run $AGENT_BINARY to $DEST"
mkdir -p $DEST || exit 1

# Copy binary
cp $AGENT_BINARY $DEST

# Copy libs
mkdir -p $DEST/libs
if ls -U libs/*.so* &> /dev/null; then
    cp libs/*.so* $DEST/libs
fi 

# Copy skill files
mkdir -p $DEST/skills
cp skills/*.skl $DEST/skills

# Copy parameter files
mkdir -p $DEST/paramfiles
cp paramfiles/*.txt $DEST/paramfiles

# Copy start/kill scripts
cp start.sh start_penalty_goalie.sh start_penalty_kicker.sh start_gazebo.sh kill.sh $DEST
