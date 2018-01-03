#!/bin/bash
#
# This script runs the simspark soccer simulator and our team against an
# opponent.  The start-stats.sh start script is called to run our team so that
# the agents use the RecordStatsBehavior and collect game statistics.
# Statistics are written to an output file.
#
# Usage: start-game-me.sh <stats_output_file> <opponent_dir> <left | right>

if [ "$#" -ne 3 ]; then
  echo "Usage: $0 <stats_output_file> <opponent_dir> <left | right>"
  exit 1
fi

SCRIPT=$(readlink -f $0)
SCRIPT_DIR=`dirname $SCRIPT`

OUTPUT_FILE=$1
OPPONENT_AGENT_DIR=$2
AGENT_SIDE=$3

HOST=`hostname`

start_time=`date +%s`

killall -9 rcssserver3d

# Start server
rcssserver3d &

PID=$!

sleep 5

cd $SCRIPT_DIR                                                                

if [ "$AGENT_SIDE" == "right" ]
then 
	cd $OPPONENT_AGENT_DIR
	./start.sh 127.0.0.1
	sleep 1
fi

cd $SCRIPT_DIR
./start-stats.sh 127.0.0.1 $OUTPUT_FILE
sleep 1

if [ "$AGENT_SIDE" != "right" ] # Should be left
then 
	cd $OPPONENT_AGENT_DIR
	./start.sh 127.0.0.1
	sleep 1
fi


sleep 10
perl $SCRIPT_DIR/kickoff.pl # kickoff to start 

max_wait_time_secs=1500

while [ ! -f $OUTPUT_FILE ] && [ `expr \`date +%s\` - $start_time` -lt $max_wait_time_secs ]
do 
  if ! kill -0 $PID &> /dev/null
  then
      echo "Server died so shutting down."
      break
  fi 
  sleep 5
done 

if [ ! -f $OUTPUT_FILE ]
then
  echo "Timed out while waiting for game to complete, current wait time is `expr \`date +%s\` - $start_time` seconds."
else
  echo "Completed with a wait time of `expr \`date +%s\` - $start_time` seconds."
fi

sleep 1

echo "Killing agent team"
cd $SCRIPT_DIR/..
./kill.sh

echo "Killing opponent team"
cd $OPPONENT_AGENT_DIR
./kill.sh

echo "Killing Simulator"
kill -s 2 $PID

sleep 2
echo "Finished"
