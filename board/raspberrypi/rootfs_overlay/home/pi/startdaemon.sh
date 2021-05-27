#!/bin/sh
sudo ifdown can0
sudo ifup can0
cd /home/pi/daemons
./Haltechd
