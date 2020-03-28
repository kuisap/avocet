#!/bin/bash
set -eux
git submodule update --init
cd console
catkin_make . -DCMAKE_BUILD_TYPE=Debug
