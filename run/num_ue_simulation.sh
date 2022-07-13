
#!/bin/bash

if [ "$#" -ne 2 ]; then
	echo "Usage: ../run/recursive_simulation.sh <index_start> <index_end>"
	exit
fi

START=$1
END=$2

datarate=5
for num_user in 10 80;
do
	for depth in $(seq 1 2 5)
	do
		result_dir="result/datarate_$datarate/user_$num_user/depth_$depth"
		python iterative_simulation.py -t $depth --index_start $START --index_end $END --num_user $num_user --datarate $datarate --result_path $result_dir
	done
	echo "------------------------------------------------------"
done
