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

for datarate in $(seq 0 2 10)
do
	for window in $WINDOWS
	do
		if [ $window -eq 0 ]; then
			continue
		fi
		END=`expr $START + $NUM_EXP`
		tmux send-keys -t $sess:$window "python no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END --datarate $datarate" Enter
		START=$END
	done
done

for datarate in $(seq 1 2 9)
do
	for window in $WINDOWS
	do
		if [ $window -eq 0 ]; then
			continue
		fi
		END=`expr $START + $NUM_EXP`
		tmux send-keys -t $sess:$window "python no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END --datarate $datarate" Enter
		START=$END
	done
done
