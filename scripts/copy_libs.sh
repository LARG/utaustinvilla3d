#!/bin/bash
# Copies shared objects of executable to a directory
# Defaults to copying binary's shared objects to libs directory
# Usage: copy_libs.sh [executable] [destination]
# Suggested: copy_libs.sh [binary] [libs]


if [ "$#" -gt 2 ]; then
  echo "Usage: $0 [executable] [destination]"
  echo "Suggested: $0 [binary] [libs]"
  exit 1
fi

EXEC=agentspark
DEST=libs

if [ $1 ]; then
    DIR_EXEC="$( cd "$( dirname "$1" )" && pwd )" || exit 1
    EXEC=$DIR_EXEC/$(basename $1)
fi

if [ $2 ]; then
    DIR_DEST="$( cd "$( dirname "$2" )" && pwd )" || exit 1
    DEST=$DIR_DEST/$(basename $2)
fi

DIR_SCRIPT="$( cd "$( dirname "$0" )" && pwd )" 
cd $DIR_SCRIPT/..

echo "Copying $EXEC shared objects to $DEST"
mkdir -p $DEST || exit 1
ldd $EXEC | grep "=> /" | awk '{print $3}' | xargs -I '{}' cp '{}' $DEST
    
