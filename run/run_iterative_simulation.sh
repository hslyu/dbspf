#!/bin/bash

if [ "$#" -ne 5 ]; then
	echo "Usage: ../run/run_iterative_simulation.sh <depth> <index_start> <index_end> <num_user> <datarate>"
	exit
fi
num_user=$4
result_dir="./result/result-depth_$1-user_$num_user"1
python iterative_simulation.py -t $1 --index_start $2 --index_end $3 --num_user $num_user --datarate $5 --result_path $result_dir
