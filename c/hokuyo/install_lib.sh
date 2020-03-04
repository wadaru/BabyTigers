#!/bin/bash
hg clone http://hg.code.sf.net/p/urgnetwork/urg_library 
cd urg_library
make
sudo make install

