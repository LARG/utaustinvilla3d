#!/bin/bash
#
# UT Austin Villa start penalty goalie script for 3D Simulation Competitions
#

AGENT_BINARY=agentspark
BINARY_DIR="."
LIBS_DIR="./libs"
export LD_LIBRARY_PATH=$LIBS_DIR:$LD_LIBRARY_PATH

team="UTAustinVilla_Base"
host="localhost"
port=3100
paramsfile=paramfiles/defaultParams.txt
mhost="localhost"

#killall -9 $AGENT_BINARY

usage()
{
  (echo "Usage: $0 [options]"
   echo "Available options:"
   echo "  --help                       prints this"
   echo "  HOST                         specifies server host (default: localhost)"
   echo "  -p, --port PORT              specifies server port (default: 3100)"
   echo "  -t, --team TEAMNAME          specifies team name"
   echo "  -mh, --mhost HOST            IP of the monitor for sending draw commands (default: localhost)"
   echo "  -pf, --paramsfile FILENAME   name of a parameters file to be loaded (default: paramfiles/defaultParams.txt)") 1>&2
}


fParsedHost=false
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

opt="${opt} --host=${host} --port ${port} --team ${team} ${paramsfile_args} --mhost=${mhost}"

DIR="$( cd "$( dirname "$0" )" && pwd )" 
cd $DIR

"$BINARY_DIR/$AGENT_BINARY" $opt --unum 1 --type 4 --paramsfile paramfiles/defaultParams_t4.txt --pkgoalie &#> /dev/null &
#"$BINARY_DIR/$AGENT_BINARY" $opt --unum 1 --type 4 --paramsfile paramfiles/defaultParams_t4.txt --pkgoalie > stdoutgk 2> stderrgk &

sleep 2
