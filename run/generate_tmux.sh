if [ "$#" -ne 8 ];
then 
	echo "usage: $0 <session_name> <NUM_SESS> <shell_path> <START_INDEX> <EXP_STEP> <NUM_UE> <DATARATE> <BANDWIDTH>"
	exit
fi	

SESS=$1
NUM_SESS=$2
SHELL_PATH=$3
START=$4
EXP_STEP=$5
NUM_UE=$6
DATARATE=$7
BANDWIDTH=$8
WINDOWS=$(seq 0 $NUM_SESS)

# tmux new-session -d -s $SESS
for window in $WINDOWS
do
	if [ $window -eq 0 ]; then
		continue
	fi
	END=`expr $START + $EXP_STEP`
	# tmux new-window -t $SESS:$window
	tmux send-keys -t $SESS:$window "ca drone" Enter
	tmux send-keys -t $SESS:$window "cd ~/dbspf" Enter
	tmux send-keys -t $SESS:$window "$SHELL_PATH $START $END $NUM_UE $DATARATE $BANDWIDTH" Enter
	START=$END
done
