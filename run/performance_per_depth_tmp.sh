#!/bin/bash

if [ $# -lt 3 ]
then
	echo "Depth, index_start and index_end should be specified. Usage: $0 <DEPTH> <env_index_start> <env_index_end>"
	exit
fi

DEPTH="$1"
INDEX_START="$2"
INDEX_END="$3"

if [ ! -d './result' ]
then
	mkdir result
fi


for i in $4
do
	NUM_USER=$(($i * 5))
	RESULT_DIR="$HOME/dbspf/result/result-depth_$DEPTH-user_$NUM_USER/"
    for ENV_INDEX in $(seq $INDEX_START $((INDEX_END-1)))
    do
        FILENAME="env_$(printf "%04d" $ENV_INDEX)-depth_$DEPTH-ue_$NUM_USER.json"
        FILE_PATH=$RESULT_DIR$FILENAME
        if [ ! -f $FILE_PATH ];
        then
            echo "File does not exist : $FILE_PATH"
            python iterative_simulation.py -t $DEPTH --num_user $NUM_USER --result_path $RESULT_DIR --index_start $ENV_INDEX --index_end $((ENV_INDEX+1))
        fi
    done
	echo "Computation for depth $DEPTH and user $NUM_USER is done."
	echo "----------------------------------------------------"
done
