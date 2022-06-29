#!/bin/bash

if [ "$#" -ne 2 ];
then 
	echo "usage: $0 <NUM_EXP> <START_INDEX>"
	exit
fi	

sess="PF"

NUM_SESS=10
NUM_EXP=$1
START=$2
WINDOWS=$(seq 0 $NUM_SESS)

for window in $WINDOWS
do
	END=`expr $START + $NUM_EXP`
	if [ $window -eq 0 ]; then
		continue
	fi

	if [ $window -lt 5 ]; then
		tmux send-keys -t $sess:$window "python no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END --datarate 5 --result_path result/datarate_5" Enter
	else
		tmux send-keys -t $sess:$window "python no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END --datarate 10 --result_path result/datarate_5" Enter
	fi
	START=$END
done
