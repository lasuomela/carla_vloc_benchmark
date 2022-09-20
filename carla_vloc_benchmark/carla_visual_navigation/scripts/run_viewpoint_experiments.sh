#!/bin/bash
autopilot=False
odometry=False
repetitions=5
exactname=False


display_help() {
	echo "Arguments:" >&2
	echo "	-e	Experiment name		Name of the expriment(s)			(Required)"
	echo "	-t	Town name								(Required)"
	echo "	-a	Autopilot		Run autopilot experiments			(Optional)	Default value: False"
	echo "	-o	Odometry 		Run odometry experiment				(Optional)	Default value: False"
	echo "	-r	Repetitions		Number of executions for each experiment	(Optional)	Default value: 5"
	echo "	-n	Exact name 		Experiment name parameter uses exact naming	(Optional)	Default value: False"
	echo "	-h	Help"
	exit 1
}

if [ "$1" == "-h" ]; then
	display_help
fi

while getopts e:t:a:o:r:n: flag
do
	case "$flag" in
		e) experiment=${OPTARG};;
		t) town=${OPTARG};;
		a) autopilot=${OPTARG};;
		o) odometry=${OPTARG};;
		r) repetitions=${OPTARG};;
		n) exactname=${OPTARG};;
		*) 
			echo 'Error in command line parsing' >&2
			display_help
	esac
done

shift "$(( OPTIND - 1 ))"

if [ -z "$experiment" ] || [ -z "$town" ]; then
	echo 'Missing required -e  or -t' >&2
	display_help
fi

echo "experiment $experiment, town $town, autopilot $autopilot odometry $odometry repetitions $repetitions exactname $exactname"
tmux new -d -s session_test
tmux send-keys -t session_test.0 "python run_experiments.py scenario --experiment $experiment --town $town --autopilot $autopilot --odometry $odometry --repetitions $repetitions --exact-name $exactname" ENTER
tmux a -t session_test
