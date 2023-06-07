#!/bin/bash

for i in $(seq 1 9); do
    ./main "./Fleury/$i-in.txt" "./Fleury/$i-out.txt"
done
