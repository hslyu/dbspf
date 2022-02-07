#!/usr/bin/env bash

OUTPUT_DIR='result/data_datarate_max_data'
python environment_generator.py\
	--output_dir $OUTPUT_DIR\
	--num_iteration 200\
	--datarate 0 0\
	--num_ue 100\
	--args_filename args_datarate-0.json

for i in $(seq 0 100)
do
	DATARATE=$i
	python environment_generator.py\
		--output_dir $OUTPUT_DIR\
		--num_iteration 200\
		--num_ue 100\
		--datarate $DATARATE $DATARATE\
		--args_filename args_datarate-$DATARATE.json\
		--generate_args_only True
done
