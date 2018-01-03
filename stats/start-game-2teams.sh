#!/bin/bash
#
# This script runs the simspark soccer simulator and two teams.  It also
# runs gameMonitor.py to collects game statistics.  Statistics are written to 
# an output file
#
# Usage: start-game-2teams.sh <stats_output_file> <team1_dir> <team2_dir> <left | right>

if [ "$#" -ne 4 ]; then
  echo "Usage: $0 <stats_output_file> <team1_dir> <team2_dir> <left | right>"
  exit 1
fi

SCRIPT=$(readlink -f $0)
SCRIPT_DIR=`dirname $SCRIPT`

OUTPUT_FILE=$1
OPPONENT_AGENT_DIR=($2 $3)
AGENT_SIDE=$4

HOST=`hostname`

start_time=`date +%s`

killall -9 rcssserver3d

# Start server
rcssserver3d &

PID=$!

sleep 5

cd $SCRIPT_DIR                                                                
./gameMonitor.py $OUTPUT_FILE $HOST &


if [ "$AGENT_SIDE" == "right" ]
then 
	cd ${OPPONENT_AGENT_DIR[1]}
	./start.sh 127.0.0.1
	sleep 1
fi

cd ${OPPONENT_AGENT_DIR[0]}
./start.sh 127.0.0.1
sleep 1

if [ "$AGENT_SIDE" != "right" ] # Should be left
then 
	cd ${OPPONENT_AGENT_DIR[1]}
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

echo "Killing agent teams"
for i in {0..1}; do
    cd ${OPPONENT_AGENT_DIR[$i]}
    ./kill.sh
done

echo "Killing Simulator"
kill -s 2 $PID

sleep 2
echo "Finished"
