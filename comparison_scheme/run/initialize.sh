#!/bin/bash

if [ "$#" -eq 0 ];
then 
	echo "usage: $0 <session name> <number of sessions>"
	exit
fi	

sess=$1
dir="/home/hslyu/dbspf/comparison_scheme"

tmux new-session -d -s "$sess" -c "$dir"

if [ -z "$2" ]
then
	NUM_SESS = $2
else
	NUM_SESS = 10
fi

WINDOWS=$(seq 0 $NUM_SESS)
for window in $WINDOWS
do
	if [ $window -eq 0 ]; then
		continue
	fi
    tmux new-window -t $sess:$window
	tmux send-keys -t $sess:$window "conda activate drone" Enter
done
