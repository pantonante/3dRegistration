#!/bin/sh

FGR=FastGlobalRegistration/build/FastGlobalRegistration
FLAGS='-c -n 10000 --max-corr-dist 0.0001'

DATASET_DIR=$1

shift #let's take all the command line arguments after the first
echo "|    Running..."
echo "|    $FGR -p $DATASET_DIR/ptCloud_P.pcd -q $DATASET_DIR/ptCloud_Q.pcd $FLAGS $@"
echo ""
$FGR -p $DATASET_DIR/ptCloud_P.pcd -q $DATASET_DIR/ptCloud_Q.pcd $FLAGS $@
echo
echo "Expected transformation matrix:"
more $DATASET_DIR/trans.txt