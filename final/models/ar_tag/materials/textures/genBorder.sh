#!/bin/bash
for f in MarkerData*.png
do
   convert $f -bordercolor White -border 150x150 border_$f
#  convert border_$f -flop border_$f
done
