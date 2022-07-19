#!/bin/bash

if [ "$#" -ne 2 ];
then 
	echo "usage: $0 <NUM_EXP> <START_INDEX>"
	exit
fi	


NUM_EXP=$1
START=$2
sess="PF"

NUM_SESS=25
WINDOWS=$(seq 0 $NUM_SESS)

dir="~/dbspf/comparison_scheme"
for window in $WINDOWS
do
	if [ $window -eq 0 ]; then
		continue
	fi
	END=`expr $START + $NUM_EXP`
	tmux send-keys -t $sess:$window "for ((UE=80;UE>=10;UE-=10)); do python $dir/no_GBS_iterative_ga_optimizer.py --index_start $START --index_end $END --num_ue \$UE; done" Enter
	START=$END
done
