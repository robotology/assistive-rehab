#!/bin/bash

#This script converts ipynb notebook file into html

FILENAME=$(yarp resource --context AssistiveRehab --from report.ipynb | awk '{gsub(/\"/,"")};1')

#Convert to notebook
echo "Converting $FILENAME to notebook..."
eval "jupyter nbconvert --execute --inplace $FILENAME"

#Convert to html
echo "Converting $FILENAME to html..."
eval "jupyter nbconvert --to html $FILENAME"

echo "Done"
