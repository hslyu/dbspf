#!/bin/bash

num_user=$4
result_dir="./result/result-depth_$1-user_$num_user"
python main.py -t $1 --index_start $2 --index_end $3 --num_user $num_user --result_path $result_dir
