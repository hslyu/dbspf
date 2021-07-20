#!/bin/bash

if [ -z $1 ]
then
	echo "Depth should be specified. Usage: $0 {DEPTH} {index_start} {index_end}"
	exit
fi

if [ ! -d './result' ]
then
	mkdir result
fi

for i in {2..11}
do
	num_user=$(($i * 5))
	result_dir="./result/result-depth_$1-user_$num_user"
	python main.py -t $1 --num_user $num_user --result_path $result_dir --index_start $2 --index_end $3
	echo "Computation for depth $1 and user $num_user is done."
	echo "----------------------------------------------------"
done
