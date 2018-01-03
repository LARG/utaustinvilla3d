#!/bin/bash
#
# Start script for agents collecting game statistics
#
# Usage: start-stats.sh <host> <stats_output_file>

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <host> <stats_output_file>"
  exit 1
fi

AGENT_BINARY=agentspark
BINARY_DIR="."
LIBS_DIR="./libs"
NUM_PLAYERS=11

team="stats"
host=$1
port=3100
paramsfile=paramfiles/defaultParams.txt
mhost="localhost"

DIR_SCRIPT="$( cd "$( dirname "$0" )" && pwd )"
cd $DIR_SCRIPT/..
export LD_LIBRARY_PATH=$LIBS_DIR:$LD_LIBRARY_PATH
 
DIR_OUTPUT="$( cd "$( dirname "$2" )" && pwd )"
OUTPUT_FILE=$DIR_OUTPUT/$(basename $2)

paramsfile_args="--paramsfile ${paramsfile}"

opt="${opt} --host=${host} --port ${port} --team ${team} ${paramsfile_args} --mhost=${mhost} --recordstats --experimentout ${OUTPUT_FILE}"

DIR="$( cd "$( dirname "$0" )" && pwd )" 
cd $DIR

for ((i=1;i<=$NUM_PLAYERS;i++)); do
    case $i in
	1|2)
	    echo "Running agent No. $i -- Type 0"
	    "$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 0 --paramsfile paramfiles/defaultParams_t0.txt &#> /dev/null &
	    #"$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 0 --paramsfile paramfiles/defaultParams_t0.txt > stdout$i 2> stderr$i &
	    ;;
	3|4)
	    echo "Running agent No. $i -- Type 1"
	    "$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 1 --paramsfile paramfiles/defaultParams_t1.txt &#>  /dev/null &
	    #"$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 1 --paramsfile paramfiles/defaultParams_t1.txt > stdout$i 2> stderr$i &
	    ;;
	5|6)
	    echo "Running agent No. $i -- Type 2"
	    "$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 2 --paramsfile paramfiles/defaultParams_t2.txt &#> /dev/null &
	    #"$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 2 --paramsfile paramfiles/defaultParams_t2.txt > stdout$i 2> stderr$i &
	    ;;
	7|8)
	    echo "Running agent No. $i -- Type 3"
	    "$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 3 --paramsfile paramfiles/defaultParams_t3.txt &#> /dev/null &
	    #"$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 3 --paramsfile paramfiles/defaultParams_t3.txt > stdout$i 2> stderr$i &
	    ;;
	*)
	    echo "Running agent No. $i -- Type 4"
	    "$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 4 --paramsfile paramfiles/defaultParams_t4.txt &#> /dev/null &
	    #"$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 4 --paramsfile paramfiles/defaultParams_t4.txt > stdout$i 2> stderr$i &
	    ;;
	
    esac
    sleep 1
done


