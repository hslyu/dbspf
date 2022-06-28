#!/bin/bash

if [ "$#" -ne 2 ];
then 
	echo "usage: $0 <NUM_EXP> <START_INDEX>"
	exit
fi	

sess="PF"
dir="/home/hslyu/dbspf/comparison_scheme"

NUM_SESS=10
NUM_EXP=$1
START=$2
WINDOWS=$(seq 0 $NUM_SESS)
for window in $WINDOWS
do
	if [ $window -eq 0 ]; then
		continue
	fi
	END=`expr $START + $NUM_EXP`
	tmux send-keys -t $sess:$window "conda activate drone" Enter
	tmux send-keys -t $sess:$window "python no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END" Enter
	START=$END
done
