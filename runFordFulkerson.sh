#!/bin/bash

for i in $(seq 1 10); do
    ./main "./FordFulkerson/$i-in.txt" "./FordFulkerson/$i-out.txt"
done
