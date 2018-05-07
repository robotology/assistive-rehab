#!/bin/bash

#This script converts ipynb notebook file into html

FILENAME=$(yarp resource --context AssistiveRehab --from report.ipynb | awk '{gsub(/\"/,"")};1')
CURR_DIR=$PWD

#Convert to notebook
echo "Converting $FILENAME to notebook..."
eval "jupyter nbconvert --execute --output-dir $CURR_DIR --inplace $FILENAME"

#Convert to html
echo "Converting $FILENAME to html..."
eval "jupyter nbconvert --output-dir $CURR_DIR --to html $FILENAME"

echo "Done"
