#!/bin/bash
# My first script

cd sample
python ../clean_sample.py
cmake . -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cd ..