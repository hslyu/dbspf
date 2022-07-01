#!/bin/bash

if [ "$#" -ne 3 ];
then 
	echo "usage: $0 <NUM_EXP> <START_INDEX> <DATARATE>"
	exit
fi	

sess="PF"

NUM_EXP=$1
START=$2
DATARATE=$3

NUM_SESS=10
WINDOWS=$(seq 0 $NUM_SESS)

for window in $WINDOWS
do
	if [ $window -eq 0 ]; then
		continue
	fi
	END=`expr $START + $NUM_EXP`
	tmux send-keys -t $sess:$window "python no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END --datarate $DATARATE" Enter
	START=$END
done
