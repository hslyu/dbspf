#!/usr/bin/env bash
mkdir result_datarate
NUM_USER=20

for i in {1..1}
do
	DATARATE=$((i*5))
	python iterative_simulation.py\
		--tree_depth 2\
		--env_path $PWD/data_required_datarate\
		--env_args_filename args_datarate-$DATARATE.json\
		--result_path $PWD/result_datarate/datarate_$DATARATE\
		--num_user $NUM_USER\
		--index_start 0\
		--index_end 1000\
		--datarate_experiment True
done
