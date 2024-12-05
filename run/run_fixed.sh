#!/bin/bash

for rate in $(seq 0 1 10)
do
	python iterative_simulation.py --index_start $1 --index_end $2 --num_user 20 --datarate $rate --mode fixed
done
