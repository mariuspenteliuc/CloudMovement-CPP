#!/bin/sh
for i in 4
do
    echo "\nNumber of threads = $i\n"
    time ./build/CloudMovement-CPP $i
done