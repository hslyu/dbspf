#!/bin/bash

if [ "$#" -ne 2 ]; then
	echo "Usage: ../run/recursive_simulation.sh <index_start> <index_end>"
	exit
fi

START=$1
END=$2

#for num_user in $(seq 10 10 80)
#do
#	for datarate in $(seq 0 15)
#	do
#		for depth in $(seq 1 7)
#		do
#			result_dir="./result/datarate_$datarate/result-depth_$depth-user_$num_user"
#			read test <<< $(python iterative_simulation.py -t $depth --index_start $START --index_end $END --num_user $num_user --datarate $datarate --result_path $result_dir)
#			echo "$test"
#		done
#		echo "------------------------------------------------------"
#	done
#done


num_user=20
for datarate in $(seq 0 4)
do
	for depth in $(seq 1 8)
	do
		result_dir="./result/datarate_$datarate/result-depth_$depth-user_$num_user"
		read test <<< $(python iterative_simulation.py -t $depth --index_start $START --index_end $END --num_user $num_user --datarate $datarate --result_path $result_dir)
		echo "$test"
	done
	echo "------------------------------------------------------"
done
