#!/usr/bin/env bash
mkdir result_datarate
NUM_USER=40
DATARATE_INDEX_START=$1
DATARATE_INDEX_END=$2
if [ $# -lt 2 ]
then
	echo "Start and end index for datarate should be specified. Usage: $0 <DATARATE_INDEX_START> <DATARATE_INDEX_END>"
	exit
fi

for i in {1..20}
do
	DATARATE=$((i*5))
	echo "Iteration for datarate $DATARATE"
	python iterative_simulation.py\
		--tree_depth 2\
		--env_path $PWD/data_required_datarate\
		--env_args_filename args_datarate-$DATARATE.json\
		--result_path $PWD/result_datarate/datarate_$DATARATE\
		--num_user $NUM_USER\
		--index_start 0\
		--index_end 100\
		--datarate_experiment True
done
