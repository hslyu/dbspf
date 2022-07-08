#!/bin/bash

if [ "$#" -ne 3 ];
then 
	echo "usage: $0 <NUM_EXP> <START_INDEX> <datarate>"
	exit
fi	

sess="PF$3"

NUM_EXP=$1
START=$2
datarate=$3

NUM_SESS=10
WINDOWS=$(seq 0 $NUM_SESS)

num_user=20
for window in $WINDOWS
do
	if [ $window -eq 0 ]; then
		continue
	fi
	END=`expr $START + $NUM_EXP`
	tmux send-keys -t $sess:$window "cd ~/dbspf" Enter
	for depth in $(seq 1 7)
	do
		result_dir="result/datarate_$datarate/user_$num_user/depth_$depth"
		tmux send-keys -t $sess:$window "python ~/dbspf/iterative_simulation.py -t $depth --index_start $START --index_end $END --num_user $num_user --datarate $datarate --result_path $result_dir" Enter
	done
	START=$END
done
