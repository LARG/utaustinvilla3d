#!/bin/bash
#
# UT Austin Villa fat proxy start script for 3D Simulation Competitions
#


AGENT_BINARY=agentspark
BINARY_DIR="."
LIBS_DIR="./libs"
NUM_PLAYERS=11

team="UTAustinVilla_Base"
host="localhost"
port=3100
paramsfile=paramfiles/defaultParams.txt
mhost="localhost"


export LD_LIBRARY_PATH=$LIBS_DIR:$LD_LIBRARY_PATH


usage()
{
  (echo "Usage: $0 [options]"
   echo "Available options:"
   echo "  --help                       prints this"
   echo "  HOST                         specifies server host (default: localhost)"
   echo "  -p, --port PORT              specifies fat proxy server port"
   echo "  -t, --team TEAMNAME          specifies team name"
   echo "  -mh, --mhost HOST            IP of the monitor for sending draw commands (default: localhost)"
   echo "  -pf, --paramsfile FILENAME   name of a parameters file to be loaded (default: paramfiles/defaultParams.txt)") 1>&2
}


fParsedHost=false
fParsedPort=false
paramsfile_args="--paramsfile ${paramsfile}"

while [ $# -gt 0 ]
do
  case $1 in

    --help)
      usage
      exit 0
      ;;

    -mh|--mhost)
      if [ $# -lt 2 ]; then
        usage
        exit 1
      fi
      mhost="${2}"
      shift 1
      ;;

    -p|--port)
      if [ $# -lt 2 ]; then
        usage
        exit 1
      fi
      port="${2}"
      fParsedPort=true
      shift 1
      ;;

    -t|--team)
      if [ $# -lt 2 ]; then
        usage
        exit 1
      fi
      team="${2}"
      shift 1
      ;;

    -pf|--paramsfile)
      if [ $# -lt 2 ]; then
        usage
        exit 1
      fi
      DIR_PARAMS="$( cd "$( dirname "$2" )" && pwd )"
      PARAMS_FILE=$DIR_PARAMS/$(basename $2)
      paramsfile_args="${paramsfile_args} --paramsfile ${PARAMS_FILE}"
      shift 1
      ;;
    *)
      if $fParsedHost;
      then
        echo 1>&2
        echo "invalid option \"${1}\"." 1>&2
        echo 1>&2
        usage
        exit 1
      else
        host="${1}"
	fParsedHost=true
      fi
      ;;
  esac

  shift 1
done

if ! $fParsedPort
then
  echo "fat proxy port must be specified: -p, --port PORT" 1>&2
  exit 1
fi


opt="${opt} --host=${host} --port ${port} --team ${team} ${paramsfile_args} --mhost=${mhost} --fatproxy"

DIR="$( cd "$( dirname "$0" )" && pwd )" 
cd $DIR

for ((i=1;i<=$NUM_PLAYERS;i++)); do
    case $i in
	2|7)
	    echo "Running agent No. $i -- Type 0"
	    "$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 0 --paramsfile paramfiles/defaultParams_t0.txt &#> /dev/null &
	    #"$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 0 --paramsfile paramfiles/defaultParams_t0.txt > stdout$i 2> stderr$i &
	    ;;
	-1)
	    echo "Running agent No. $i -- Type 1"
	    "$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 1 --paramsfile paramfiles/defaultParams_t1.txt &#>  /dev/null &
	    #"$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 1 --paramsfile paramfiles/defaultParams_t1.txt > stdout$i 2> stderr$i &
	    ;;
	8|9)
	    echo "Running agent No. $i -- Type 2"
	    "$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 2 --paramsfile paramfiles/defaultParams_t2.txt &#> /dev/null &
	    #"$BINARY_DIR/$AGENT_BINARY" $opt --unum $i --type 2 --paramsfile paramfiles/defaultParams_t2.txt > stdout$i 2> stderr$i &
	    ;;
	-1)
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


