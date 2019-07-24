#!/bin/bash

cd src/panda/src/classes

touch "$1.cpp"
cd ../..
cd include
touch "$1.h"