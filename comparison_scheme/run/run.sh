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

WINDOWS=$(seq 1 5)
for window in $WINDOWS
do
	END=`expr $START + $NUM_EXP`
	tmux send-keys -t $sess:$window "python ~/dbspf/comparison_scheme/no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END --datarate 5 --result_path ~/dbspf/comparison_scheme/result/datarate_5" Enter
	START=$END
done

START=$2
WINDOWS=$(seq 6 10)
for window in $WINDOWS
do
	END=`expr $START + $NUM_EXP`
	tmux send-keys -t $sess:$window "python ~/dbspf/comparison_scheme/no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END --datarate 10 --result_path ~/dbspf/comparison_scheme/result/datarate_10" Enter
	START=$END
done
