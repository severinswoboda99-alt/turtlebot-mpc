#!/bin/bash

DIR=$(dirname "$0")
cd $DIR/..

ament_uncrustify --reformat aruco_markers aruco_markers_msgs