#!/usr/bin/env bash
NUM_USER=40
DATARATE_INDEX_START=$((5*$1))
DATARATE_INDEX_END=$((DATARATE_INDEX_START+4))
if [ $# -lt 1 ]
then
	echo "Session number should be specified. Usage: $0 <DATARATE_INDEX_START> <DATARATE_INDEX_END>"
	exit
fi

for i in $(seq $DATARATE_INDEX_START $DATARATE_INDEX_END)
do
	DATARATE=$i
	echo "Iteration for datarate $DATARATE"
	python iterative_simulation.py\
		--tree_depth 2\
		--env_path $PWD/result/result_datarate_max_data/data_datarate_max_data_50-150\
		--env_args_filename args_datarate-$DATARATE.json\
		--result_path $PWD/result/result_datarate_max_data/max_data_150/datarate_$DATARATE\
		--num_user $NUM_USER\
		--index_start 0\
		--index_end 200\
		--datarate_experiment True
done
