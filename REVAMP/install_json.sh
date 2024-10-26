#!/bin/bash
rm -rf json 
git clone https://github.com/nlohmann/json.git
cd json
mkdir build
cd build
cmake ../
make -j2
sudo make install
