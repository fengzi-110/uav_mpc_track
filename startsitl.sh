#!/bin/bash
cd ~/git/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=127.0.0.1:14551
