cd /home/pi/PowerTune
git pull
cd /opt/PowerTune
qmake /home/pi/PowerTune/PowertuneQMLGui.pro
make -j4
