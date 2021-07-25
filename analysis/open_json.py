#!/usr/bin/env python3

# Depth first search(DFS) based UAV base station simulation code.
# Author : Hyeonsu Lyu, POSTECH, Korea
# Contact : hslyu4@postech.ac.kr
import json

def open_json(file_path):
    with open(file_path, encoding='utf-8') as f:
        result = json.load(f)
    return result
