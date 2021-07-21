#!/bin/bash

if [ $# -lt 3 ]
then
	echo "Depth, index_start and index_end should be specified. Usage: $0 {DEPTH} {index_start} {index_end}"
	exit
fi

DEPTH="$1"
INDEX_START="$2"
INDEX_END="$3"

if [ ! -d './result' ]
then
	mkdir result
fi


for i in {2..11}
do
	num_user=$(($i * 5))
	result_dir="./result/result-depth_$DEPTH-user_$num_user"
	python main.py -t $DEPTH --num_user $num_user --result_path $result_dir --index_start $INDEX_START --index_end $INDEX_END
	echo "Computation for depth $DEPTH and user $num_user is done."
	echo "----------------------------------------------------"
done
