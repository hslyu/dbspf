#!/bin/bash

sess="PF"
dir="/home/hslyu/dbspf/comparison_scheme"

tmux new-session -d -s "$sess" -c "$dir"

NUM_EXP=$1
START=$2
WINDOWS=$(seq 0 10)
for window in $WINDOWS
do
	if [ $window -eq 0 ]; then
		continue
	fi
	END=`expr $START + $NUM_EXP`
    tmux new-window -t $sess:$window
	tmux send-keys -t $sess:$window "conda activate drone" Enter
	tmux send-keys -t $sess:$window "python no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END" Enter
	START=$END
done
