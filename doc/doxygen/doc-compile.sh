#!/bin/bash

DESTINATION=generated-from-xml

# clean-up
rm -Rf doc
rm -Rf $DESTINATION

# generate doxy from xml
mkdir $DESTINATION
list=`find ../../lib ../../modules -iname *.xml | xargs`
for i in $list
do
   filename=`basename $i`
   doxyfile=${filename%%.*}
   xsltproc --output $DESTINATION/$doxyfile.dox $YARP_SOURCE_DIR/scripts/yarp-module.xsl $i
done

doxygen ./generate.txt
