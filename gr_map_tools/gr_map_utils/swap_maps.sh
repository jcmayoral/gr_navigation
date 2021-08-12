#!/bin/bash
echo "RUN"
while true
do
  rostopic pub /osm_required std_msgs/String "data: OSM" -1
  sleep 5
  rostopic pub /osm_required std_msgs/String "data: TOPO" -1
  sleep 5
done
