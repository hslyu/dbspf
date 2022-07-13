#!/bin/bash

if [ "$#" -ne 2 ];
then 
	echo "usage: $0 <NUM_EXP> <START_INDEX>"
	exit
fi	

sess="PF"

NUM_EXP=$1
START=$2

NUM_SESS=10
WINDOWS=$(seq 0 $NUM_SESS)

for window in $WINDOWS
do
	if [ $window -eq 0 ]; then
		continue
	fi
	END=`expr $START + $NUM_EXP`
	tmux send-keys -t $sess:$window "cd ~/dbspf" Enter
	tmux send-keys -t $sess:$window "run/num_ue_simulation.sh $START $END" Enter
	START=$END
done
