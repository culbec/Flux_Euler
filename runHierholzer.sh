#!/bin/bash

for i in $(seq 1 9); do
    ./main "./Hierholzer/$i-in.txt" "./Hierholzer/$i-out.txt"
done
