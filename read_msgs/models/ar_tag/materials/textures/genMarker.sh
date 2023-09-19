#!/bin/bash
for f in {0..40}
do
   rosrun ar_track_alvar createMarker $f
done
