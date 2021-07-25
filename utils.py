#!/usr/bin/env python3

# util funtions of depth first search(DFS) based UAV base station
# If you want to simulate just one environment, execute drone_basestation.py.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import os
import json

def create_dir(dir_path):
    try:#{{{
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        else:
            print(f'Directory already exists: at {dir_path}')
    except OSError:
        print('Fail to create diretory: ' +  dir_path)#}}}

def open_json(file_path):
    with open(file_path, encoding='utf-8') as f:
        result = json.load(f)
    return result
