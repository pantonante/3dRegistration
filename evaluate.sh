#!/bin/bash

# Macros
DATASET_DIR=dataset
FGR=./FastGlobalRegistration/build/FastGlobalRegistration

# Run the evaluation or 3D registration algorithms
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

for D in `find $DATASET_DIR/* -type d`
do
    echo "Evaluating 'Fast Global Registration' on: $D"
    $FGR -p $D/ptCloud_P.pcd -q $D/ptCloud_Q.pcd -r $D/FGR_report.html > $D/fgr.log
    $FGR -p $D/ptCloud_P.pcd -q $D/ptCloud_Q.pcd -c -n 12 -r $D/FGR_report_CF.html > $D/fgr_cf.log
done