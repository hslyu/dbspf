#!/bin/bash
#./plot_3d_path.py -f ../result_datarate/datarate_$2/env_$1-depth_2-ue_40.json

 for i in {1..4}
 do
	 ./plot_3d_path.py -f ../result/result-depth_$i-user_35/env_$1-depth_$i-ue_35.json
 done
