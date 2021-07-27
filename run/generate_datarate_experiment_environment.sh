#!/usr/bin/env bash

python environment_generator.py\
	--output_dir data_required_datarate\
	--num_iteration 1000\
	--datarate 5 5\
	--args_filename args_datarate-10.json
#
for i in {1..20}
do
	DATARATE=$((i*5))
	python environment_generator.py\
		--output_dir data_required_datarate\
		--num_iteration 1000\
		--datarate $DATARATE $DATARATE\
		--args_filename args_datarate-$DATARATE.json\
		--generate_args_only True
done
