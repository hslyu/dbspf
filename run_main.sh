#!/bin/bash

#python main.py -t $1 --index_start $2 --index_end $3
python3 main.py -t 4 --index_start 000 --index_end 250   &
python3 main.py -t 4 --index_start 250 --index_end 500   &
python3 main.py -t 4 --index_start 500 --index_end 750   &
python3 main.py -t 4 --index_start 750 --index_end 1000  &
python3 main.py -t 4 --index_start 1250 --index_end 1500 &
python3 main.py -t 4 --index_start 1750 --index_end 2000 &
python3 main.py -t 4 --index_start 2250 --index_end 2500 &
python3 main.py -t 4 --index_start 2750 --index_end 3000 &
python3 main.py -t 4 --index_start 3250 --index_end 3500 &
python3 main.py -t 4 --index_start 3750 --index_end 4000 &
python3 main.py -t 4 --index_start 4250 --index_end 4500 &
python3 main.py -t 4 --index_start 4750 --index_end 5000 &
