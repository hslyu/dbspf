#!/bin/bash

if [ "$#" -ne 5 ]; then
	echo "Usage: ../run/run_iterative_simulation.sh <index_start> <index_end> <num_user> <datarate> <bandwidth>"
	exit
fi
index_start=$1
index_end=$2
num_user=$3
datarate=$4
bw=$5
scheme=ga_iter

for num_user in $(seq 10 10 80)
do
	python iterative_simulation.py --index_start $1 --index_end $2 --num_user $num_user --datarate $4 --mode $scheme --bandwidth $bw \
		--result_path $HOME/storage/dbspf/bw${bw}/${scheme}-user
done
