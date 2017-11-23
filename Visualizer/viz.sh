#!/bin/bash

DIR=`dirname $0`
VISUALIZER=DIR/build/viz
$VISUALIZER -p $1/ptCloud_P.pcd -q $1/ptCloud_Q.pcd $@
