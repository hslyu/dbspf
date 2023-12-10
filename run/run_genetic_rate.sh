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
scheme=genetic

for rate in $(seq 0 1 10)
do
	python iterative_simulation.py --index_start $index_start --index_end $index_end --num_user $num_user --datarate $rate --mode $scheme --bandwidth $bw \
		--result_path $HOME/storage/dbspf/bw${bw}/${scheme}-rate
done
