#!/bin/bash

#This script converts ipynb notebook file into html

FILENAME_ITA=$(yarp resource --context AssistiveRehab --from report-ita.ipynb | tail -n1 | awk '{gsub(/\"/,"")};1')
FILENAME_ENG=$(yarp resource --context AssistiveRehab --from report-eng.ipynb | tail -n1 | awk '{gsub(/\"/,"")};1')
CURR_DIR=$PWD

#Convert to notebook
echo "Converting $FILENAME_ITA to notebook..."
eval "jupyter nbconvert --execute --output-dir $CURR_DIR --inplace $FILENAME_ITA"

echo "Converting $FILENAME_ENG to notebook..."
eval "jupyter nbconvert --execute --output-dir $CURR_DIR --inplace $FILENAME_ENG"

#Convert to html
echo "Converting $FILENAME_ITA to html..."
eval "jupyter nbconvert --output-dir $CURR_DIR --to html $FILENAME_ITA"

echo "Converting $FILENAME_ENG to html..."
eval "jupyter nbconvert --output-dir $CURR_DIR --to html $FILENAME_ENG"

echo "Done"
