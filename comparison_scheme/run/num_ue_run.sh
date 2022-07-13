#!/bin/bash

if [ "$#" -ne 3 ];
then 
	echo "usage: $0 <NUM_EXP> <START_INDEX> <NUM_USER>"
	exit
fi	


NUM_EXP=$1
START=$2
NUM_UE=$3

sess="PF"

NUM_SESS=10
WINDOWS=$(seq 0 $NUM_SESS)

dir="~/dbspf/comparison_scheme"
for window in $WINDOWS
do
	if [ $window -eq 0 ]; then
		continue
	fi
	END=`expr $START + $NUM_EXP`
	tmux send-keys -t $sess:$window "python $dir/no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END --num_ue $NUM_UE" Enter
	START=$END
done
