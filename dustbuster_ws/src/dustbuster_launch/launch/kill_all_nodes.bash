#!/bin/bash

# Get a list of all running nodes
node_list=$(ros2 node list)

# Kill each node using pkill
for node in $node_list
do
  pkill -f $node
done
