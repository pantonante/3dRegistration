#!/bin/bash

## Usage help
function print_usage(){
	echo "${0##*/} [options]"
	echo 
	echo ' -h                         print this help message'
	echo ' -v[--verbose]             verbose execution'
	echo ' -e[--exclude-dirs]=dirs    list of folder to exclude, separated by commas (e.g. dir1,dir2)'
	echo
}

# Macros
DATASET_DIR=dataset
FGR=./FastGlobalRegistration/build/FastGlobalRegistration
ADDITIONAL_FLAGS='--tuple-scale=0.9'

# Hello string!
cat << EOF
  ____  _____    _____            _     _             _   _             
 |___ \|  __ \  |  __ \          (_)   | |           | | (_)            
   __) | |  | | | |__) |___  __ _ _ ___| |_ _ __ __ _| |_ _  ___  _ __  
  |__ <| |  | | |  _  // _ \/ _  | / __| __| '__/ _  | __| |/ _ \| '_ \ 
  ___) | |__| | | | \ \  __/ (_| | \__ \ |_| | | (_| | |_| | (_) | | | |
 |____/|_____/  |_|  \_\___|\__, |_|___/\__|_|  \__,_|\__|_|\___/|_| |_|
                             __/ |                                      
                            |___/                                       

EOF

verbose_flag=""
exclude="/"

# Parsing command line arguments
for i in "$@"
do
case $i in
    -e=*|--exclude-dir=*)
	    exclude="${i#*=}"
		  exclude=${exclude//,/|} #replace commas with |
	    shift
    	;;
    -h|--help)
	  	print_usage
	  	exit 0
  		;;
    -v|--verbose)
	    verbose_flag="-v"
	    shift
    	;;
    *)
		# unknown option
    	;;
esac
done


for D in `find $DATASET_DIR/* -type d | grep -wvFE $exclude`
do
	echo "Evaluating 'Fast Global Registration' on: $D"
    $FGR -p $D/ptCloud_P.pcd -q $D/ptCloud_Q.pcd $ADDITIONAL_FLAGS $verbose_flag -o $D/FGR_trans.txt -r $D/FGR_report.html > $D/fgr.log
    $FGR -p $D/ptCloud_P.pcd -q $D/ptCloud_Q.pcd $ADDITIONAL_FLAGS $verbose_flag -c -o $D/FGR_CF_trans.txt -r $D/FGR_report_CF.html > $D/fgr_cf.log
done


